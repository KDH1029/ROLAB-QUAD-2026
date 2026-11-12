// wall_filter_node — Rgosu (2026-11-03)
//
// 벽(수직면)과 경사로/계단(비수직면)을 기하학적으로 구분해서, ego_planner의
// 장애물 입력을 "진짜 벽만"으로 좁힌다. 팀 review 브랜치(merge_tmp_20261102)의
// cloud_bridge "vertical scan" 알고리즘을 참고해서, 생명주기/TF 신규 의존성 없이
// 우리 기존 파이프라인(/velodyne_points + /Odometry)만으로 동작하게 재구성함.
//
// 원리: 방위각 1도 칸마다, 인접한 라이다 링 여러 개가 거의 같은 수평거리에
// 걸리면 그건 수직인 벽이다(여러 높이에서 똑같이 가까움). 경사로·계단 디딤면은
// 올라갈수록(링이 바뀔수록) 거리가 달라져서 이 조건에 안 걸린다 — 계단을
// 벽으로 오인하지 않음.
//
// 입력이 몸체(라이다) 좌표계라 로봇이 기울면(경사로 위) 진짜 수직인 벽도
// 기울어 보여서 오검출될 수 있다 — 그래서 /Odometry의 현재 roll/pitch로
// 먼저 수평을 맞추고(기울기만 제거, yaw는 유지) 나서 검사한다.
//
// ★ 2026-11-04 저녁 수정(라이다→몸체 외부파라미터 누락): /velodyne_points 는 cloud_bridge 가
// XT16 좌표 그대로 넘기고(z축 약 +90° 돌아가 있음, 몸체 앞 25 cm·위 15.9 cm), /Odometry 는
// FAST-LIO 의 몸체(IMU) 자세다. 예전 코드는 라이다 좌표 점에 몸체 자세를 바로 곱해서
// 벽·콘이 로봇 둘레로 약 90° 돌아간 자리에 찍혔다(17:56 녹화 재현: 실제 구조물 위 58.1% → 고친 뒤 97.9%).
// 이제 점을 먼저 extrinsic_R 로 몸체 축에 맞추고(원점은 라이다 그대로 — 수직면 판정의 광선 기하 유지),
// 출력 때 extrinsic_T 를 더한다. 값은 FAST-LIO 설정(fastlio_leg_nurigwan_derange30.yaml)과 같다.
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <stdexcept>

namespace wall_filter
{

struct Mat3
{
  double m[9];
  static Mat3 identity() { return Mat3{{1, 0, 0, 0, 1, 0, 0, 0, 1}}; }
  static Mat3 rotX(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    return Mat3{{1, 0, 0, 0, c, -s, 0, s, c}};
  }
  static Mat3 rotY(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    return Mat3{{c, 0, s, 0, 1, 0, -s, 0, c}};
  }
  static Mat3 rotZ(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    return Mat3{{c, -s, 0, s, c, 0, 0, 0, 1}};
  }
  Mat3 operator*(const Mat3 & o) const
  {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        double v = 0.0;
        for (int k = 0; k < 3; ++k) {v += m[i * 3 + k] * o.m[k * 3 + j];}
        r.m[i * 3 + j] = v;
      }
    }
    return r;
  }
  void apply(double x, double y, double z, double & ox, double & oy, double & oz) const
  {
    ox = m[0] * x + m[1] * y + m[2] * z;
    oy = m[3] * x + m[4] * y + m[5] * z;
    oz = m[6] * x + m[7] * y + m[8] * z;
  }
};

// 쿼터니언 -> roll(x축)/pitch(y축)/yaw(z축), R = Rz(yaw)*Ry(pitch)*Rx(roll) 가정 (REP103)
void quatToRPY(double w, double x, double y, double z, double & roll, double & pitch, double & yaw)
{
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(sinr_cosp, cosr_cosp);
  const double sinp = 2.0 * (w * y - z * x);
  pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

class WallFilterNode : public rclcpp::Node
{
public:
  WallFilterNode()
  : Node("wall_filter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/velodyne_points");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mission/wall_cloud");
    world_frame_ = declare_parameter<std::string>("world_frame", "camera_init");
    n_ring_ = declare_parameter<int>("n_ring", 16);
    azimuth_bins_ = declare_parameter<int>("azimuth_bins", 360);
    min_range_ = declare_parameter<double>("min_range", 0.25);
    max_range_ = declare_parameter<double>("max_range", 10.0);
    vertical_tol_m_ = declare_parameter<double>("vertical_tol_m", 0.03);
    vertical_min_rings_ = std::max<int>(2, declare_parameter<int>("vertical_min_rings", 3));
    vertical_min_span_m_ = declare_parameter<double>("vertical_min_span_m", 0.08);
    // 라이다→몸체 외부파라미터 (FAST-LIO extrinsic_R/T 와 같은 값·같은 뜻: p_body = R * p_lidar + T)
    const auto er = declare_parameter<std::vector<double>>("extrinsic_R", std::vector<double>{
      -0.0001707, -0.9999240, 0.0123287, 0.9999844, -0.0001019, 0.0055807, -0.0055790, 0.0123295, 0.9999084});
    const auto et = declare_parameter<std::vector<double>>("extrinsic_T", std::vector<double>{0.25, 0.0, 0.1588});
    if (er.size() != 9 || et.size() != 3) {throw std::runtime_error("extrinsic_R 는 9개, extrinsic_T 는 3개여야 한다");}
    for (int i = 0; i < 9; ++i) {R_ext_.m[i] = er[i];}
    for (int i = 0; i < 3; ++i) {T_ext_[i] = et[i];}

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WallFilterNode::onCloud, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&WallFilterNode::onOdom, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    RCLCPP_INFO(
      get_logger(), "wall_filter: %s(+%s) -> %s (ring=%d, az_bins=%d, tol=%.3fm, min_rings=%d, min_span=%.3fm)",
      input_topic_.c_str(), odom_topic_.c_str(), output_topic_.c_str(),
      n_ring_, azimuth_bins_, vertical_tol_m_, vertical_min_rings_, vertical_min_span_m_);
    RCLCPP_INFO(
      get_logger(), "wall_filter: 라이다→몸체 외부파라미터 R 첫 행 [%.4f %.4f %.4f] T [%.3f %.3f %.3f]",
      R_ext_.m[0], R_ext_.m[1], R_ext_.m[2], T_ext_[0], T_ext_[1], T_ext_[2]);
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    have_odom_ = true;
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    tx_ = p.x; ty_ = p.y; tz_ = p.z;
    quatToRPY(q.w, q.x, q.y, q.z, roll_, pitch_, yaw_);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!have_odom_) {return;}
    // 몸체좌표 -> 수평보정: R_wb(몸체->월드) = Rz(yaw)*Ry(pitch)*Rx(roll) 이므로,
    // p_level := Ry(pitch)*Rx(roll)*p_body 로 두면 p_world = Rz(yaw)*p_level + t 가 성립한다
    // (p_level 은 "yaw는 유지, roll·pitch는 이미 반영된" 좌표계 — 진짜 수평 기준이 됨).
    const Mat3 R_level = Mat3::rotY(pitch_) * Mat3::rotX(roll_);
    const Mat3 R_yaw = Mat3::rotZ(yaw_);
    // 점: 라이다 좌표 → (R_ext) 몸체 축 → (R_level) 수평보정. 원점은 라이다(광선 기하 그대로).
    const Mat3 R_in = R_level * R_ext_;
    // 출력: p_world = R_yaw * q + R_yaw * R_level * T_ext + t   (q = R_level * R_ext * p_lidar)
    double lvx, lvy, lvz, ofx, ofy, ofz;
    R_level.apply(T_ext_[0], T_ext_[1], T_ext_[2], lvx, lvy, lvz);
    R_yaw.apply(lvx, lvy, lvz, ofx, ofy, ofz);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> it_ring(*msg, "ring");

    const int n_az = azimuth_bins_;
    const int n_ring = n_ring_;
    const double d_az = 2.0 * M_PI / n_az;
    std::vector<float> rh(static_cast<size_t>(n_az) * n_ring, std::numeric_limits<float>::infinity());
    std::vector<float> zz(static_cast<size_t>(n_az) * n_ring, 0.0f);

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_ring) {
      const float bx = *it_x, by = *it_y, bz = *it_z;
      const uint16_t ring = *it_ring;
      if (ring >= static_cast<uint16_t>(n_ring)) {continue;}
      double lx, ly, lz;
      R_in.apply(bx, by, bz, lx, ly, lz);
      const double r = std::hypot(lx, ly);
      if (r < min_range_ || r >= max_range_) {continue;}
      int a = static_cast<int>((std::atan2(ly, lx) + M_PI) / d_az);
      a = std::clamp(a, 0, n_az - 1);
      const size_t k = static_cast<size_t>(a) * static_cast<size_t>(n_ring) + ring;
      if (static_cast<float>(r) < rh[k]) {rh[k] = static_cast<float>(r); zz[k] = static_cast<float>(lz);}
    }

    std::vector<float> out_x, out_y, out_z;
    const float tol = static_cast<float>(vertical_tol_m_);
    const float min_span = static_cast<float>(vertical_min_span_m_);
    for (int a = 0; a < n_az; ++a) {
      const float * r = &rh[static_cast<size_t>(a) * n_ring];
      const float * z = &zz[static_cast<size_t>(a) * n_ring];
      for (int i = 0; i < n_ring; ++i) {
        if (!std::isfinite(r[i])) {continue;}
        float lo = r[i], hi = r[i], zlo = z[i], zhi = z[i];
        int j = i + 1;
        for (; j < n_ring && std::isfinite(r[j]); ++j) {
          const float nlo = std::min(lo, r[j]), nhi = std::max(hi, r[j]);
          if (nhi - nlo > tol) {break;}
          lo = nlo; hi = nhi; zlo = std::min(zlo, z[j]); zhi = std::max(zhi, z[j]);
        }
        if (j - i >= vertical_min_rings_ && zhi - zlo >= min_span) {
          const double az_rad = -M_PI + (a + 0.5) * d_az;
          const double rr = 0.5 * (lo + hi);
          const double lx = rr * std::cos(az_rad), ly = rr * std::sin(az_rad);
          const double lz = 0.5 * (zlo + zhi);
          double wx, wy, wz;
          R_yaw.apply(lx, ly, lz, wx, wy, wz);
          out_x.push_back(static_cast<float>(wx + ofx + tx_));
          out_y.push_back(static_cast<float>(wy + ofy + ty_));
          out_z.push_back(static_cast<float>(wz + ofz + tz_));
          i = j - 1;
        }
      }
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = world_frame_;
    out.height = 1;
    out.width = static_cast<uint32_t>(out_x.size());
    out.is_bigendian = false;
    out.is_dense = true;
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(out_x.size());
    sensor_msgs::PointCloud2Iterator<float> ox(out, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(out, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(out, "z");
    for (size_t i = 0; i < out_x.size(); ++i, ++ox, ++oy, ++oz) {
      *ox = out_x[i]; *oy = out_y[i]; *oz = out_z[i];
    }
    pub_->publish(out);
  }

  std::string input_topic_, odom_topic_, output_topic_, world_frame_;
  int n_ring_{16}, azimuth_bins_{360};
  double min_range_{0.25}, max_range_{10.0};
  double vertical_tol_m_{0.03}, vertical_min_span_m_{0.08};
  int vertical_min_rings_{3};
  Mat3 R_ext_ = Mat3::identity();
  double T_ext_[3]{0.0, 0.0, 0.0};

  bool have_odom_{false};
  double tx_{0.0}, ty_{0.0}, tz_{0.0}, roll_{0.0}, pitch_{0.0}, yaw_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace wall_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wall_filter::WallFilterNode>());
  rclcpp::shutdown();
  return 0;
}
