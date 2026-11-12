// terrain_veto_node — 경사로·계단 위 점을 플래너 입력에서 지운다. (night_20261104, 원본 ego_planner 는 건드리지 않음)
//
// 왜: ego_planner(grid_map.cpp) 는 '점 높이 − 로봇 높이' 만 본다 → 1 m 앞 경사(약 8° 이상)·계단 디딤판이 벽이 된다.
//     10-30 녹화본: X자 구간 표본의 16.4% 에서 로봇이 실제로 걸어간 길 위에 헛장애물.
// 어떻게: 장애물을 '찾지' 않는다. 로봇 발밑과 이어진 '매끈한 면'(경사로·계단 디딤판·단상 윗면)을 찾아 그 칸의 점만 지우고
//        나머지는 그대로 넘긴다 → 못 본 곳·애매한 곳은 ego_planner 원래 규칙 그대로(보수적). 벽·라바콘은 매끈한 면이 아니라 남는다.
//   ① 높이 격자: XT16(입력 점구름) + Unitree L1(/utlidar/cloud_base) + D435i 깊이를 acc_time 동안 누적, 칸(5 cm)별 최고·최저 (로봇 z + max_h 위 = 천장 제외)
//   ② 제외 칸: 칸 안 높이 폭 > max_spread(세로 구조 = 벽), 솟음 = 최고 − 고리(0.15~0.30 m) 80% 높이 > peak_h(라바콘), 그 옆 1칸
//   ③ 통과 면: 발밑 지면 평면 근처 칸(seed_r 안) 에서 hop 안 칸으로, 높이차 ≤ max_step 이면 이어짐 (계단 한 단 OK)
//   ④ 출력: 이번 스캔 + 최신 깊이 프레임의 점 중 통과 면 칸이 아닌 것 → out_topic (같은 좌표계)
//   ⑤ 낭떠러지(11-04 cliff_*): 통과면 가장자리 칸에서 바깥 cliff_rmin~cliff_reach m 안 T 아닌 유효 칸이 cliff_drop 넘게 낮으면
//      그 가장자리 칸 중심에 장애물 점(z = 로봇 z + cliff_dz) 추가. ego 규칙은 로봇보다 낮은 점을 장애물로 안 쳐서 단(0.34 m) 옆면을
//      못 보는데, 거부권이 경사로 헛장애물을 지우면서 우연히 막혀 있던 단 가장자리가 열렸다(tv2 뚫림 최대 150칸). 원형: ramp/cliff/terrain_cliff.py
// 입력 좌표계: 점구름·오도메트리 모두 세계(camera_init) — /cloud_active·/Odometry_mux (FAST-LIO 든 백업이든)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <chrono>
#include <array>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <vector>

namespace {
struct Pt { float x, y, z; };
struct Frame { double t; std::vector<Pt> pts; };
struct Pose { double t, x, y, z, qx, qy, qz, qw; };

void quat_to_R(double x, double y, double z, double w, double R[9]) {
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - z * w);     R[2] = 2 * (x * z + y * w);
  R[3] = 2 * (x * y + z * w);     R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - x * w);
  R[6] = 2 * (x * z - y * w);     R[7] = 2 * (y * z + x * w);     R[8] = 1 - 2 * (x * x + y * y);
}

bool read_xyz(const sensor_msgs::msg::PointCloud2 & m, std::vector<Pt> & out) {
  int ox = -1, oy = -1, oz = -1;
  for (const auto & f : m.fields) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
    if (f.name == "x") ox = f.offset; else if (f.name == "y") oy = f.offset; else if (f.name == "z") oz = f.offset;
  }
  if (ox < 0 || oy < 0 || oz < 0) return false;
  const size_t n = static_cast<size_t>(m.width) * m.height;
  out.clear(); out.reserve(n);
  const uint8_t * d = m.data.data();
  for (size_t i = 0; i < n; ++i) {
    const uint8_t * p = d + i * m.point_step;
    Pt q; std::memcpy(&q.x, p + ox, 4); std::memcpy(&q.y, p + oy, 4); std::memcpy(&q.z, p + oz, 4);
    if (std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z)) out.push_back(q);
  }
  return true;
}

// ==CLIFF_BEGIN== (ramp/cliff/test_cliff_cpp.sh 가 이 구간만 떼어 파이썬 원형 terrain_cliff.cliff_edges 와 대조한다 — std 만 쓸 것)
struct CliffStats { int edge = 0, cliff = 0, empty = 0; };

// 가장자리 칸에서 중심거리 r_min~reach 안 칸 오프셋
std::vector<std::array<int, 2>> cliff_offsets(double cell, double r_min, double reach) {
  std::vector<std::array<int, 2>> o;
  const int R = static_cast<int>(std::ceil(reach / cell + 1e-9));
  for (int i = -R; i <= R; ++i)
    for (int j = -R; j <= R; ++j) {
      const double d = cell * std::hypot(static_cast<double>(i), static_cast<double>(j));
      if (d >= r_min - 1e-9 && d <= reach + 1e-9) o.push_back({i, j});
    }
  return o;
}

// n×n 격자(k = i·n + j). 가장자리 = T 칸 중 격자 안 8-이웃에 T 아닌 칸이 있는 것.
// 낭떠러지 = 가장자리 중 바깥(오프셋) T 아닌 유효 칸의 hi 가 가장자리 hi − drop 보다 낮은 게 min_low 개 이상.
// 빈 가장자리 = 바깥 T 아닌 칸이 있는데 전부 데이터 없음(세기만).
// band > 0 이면 가장자리 hi 가 발밑 지면 평면(plane6 = fx,fy,fz,nx,ny,nz; 격자 원점 ox,oy, 칸 cell)에서 ±band 안일 때만 낭떠러지.
CliffStats find_cliff(int n, const std::vector<float> & hi, const std::vector<int> & cnt, const std::vector<uint8_t> & trav,
                      const std::vector<std::array<int, 2>> & offs, double drop, int min_low,
                      std::vector<uint8_t> & cliff, std::vector<uint8_t> * edge_out = nullptr, std::vector<uint8_t> * empty_out = nullptr,
                      double band = -1.0, const double * plane6 = nullptr, double ox = 0.0, double oy = 0.0, double cell = 0.05) {
  CliffStats st;
  cliff.assign(static_cast<size_t>(n) * n, 0);
  if (edge_out) edge_out->assign(static_cast<size_t>(n) * n, 0);
  if (empty_out) empty_out->assign(static_cast<size_t>(n) * n, 0);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const int k = i * n + j;
      if (!trav[k]) continue;
      bool edge = false;
      for (int a = std::max(0, i - 1); a <= std::min(n - 1, i + 1) && !edge; ++a)
        for (int c = std::max(0, j - 1); c <= std::min(n - 1, j + 1); ++c)
          if (!trav[a * n + c]) { edge = true; break; }
      if (!edge) continue;
      ++st.edge;
      if (edge_out) (*edge_out)[k] = 1;
      const double thr = static_cast<double>(hi[k]) - drop;
      int nlow = 0, nval = 0, nnon = 0;
      for (const auto & o : offs) {
        const int a = i + o[0], c = j + o[1];
        if (a < 0 || c < 0 || a >= n || c >= n) continue;
        const int kk = a * n + c;
        if (trav[kk]) continue;
        ++nnon;
        if (!cnt[kk]) continue;
        ++nval;
        if (static_cast<double>(hi[kk]) < thr) ++nlow;
      }
      bool in_band = true;
      if (band > 0 && plane6) {
        const double x = ox + (i + 0.5) * cell, y = oy + (j + 0.5) * cell, nzc = std::max(plane6[5], 0.5);
        const double zp = plane6[2] - (plane6[3] * (x - plane6[0]) + plane6[4] * (y - plane6[1])) / nzc;
        in_band = std::fabs(static_cast<double>(hi[k]) - zp) <= band;
      }
      if (nlow >= min_low && in_band) { cliff[k] = 1; ++st.cliff; }
      else if (nval == 0 && nnon > 0) { ++st.empty; if (empty_out) (*empty_out)[k] = 1; }
    }
  return st;
}
// ==CLIFF_END==
}  // namespace

class TerrainVeto : public rclcpp::Node {
public:
  TerrainVeto() : Node("terrain_veto") {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_active");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry_mux");
    out_topic_ = declare_parameter<std::string>("out_topic", "/cloud_planner");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_rect_raw");   // "" 이면 깊이 안 씀
    l1_topic_ = declare_parameter<std::string>("l1_topic", "/utlidar/cloud_base");              // Unitree L1, base_link 좌표. "" 이면 안 씀
    acc_time_ = declare_parameter<double>("acc_time", 0.5);
    cell_ = declare_parameter<double>("cell", 0.05);
    half_ = declare_parameter<double>("half", 4.0);
    max_h_ = declare_parameter<double>("max_h", 1.10);
    max_step_ = declare_parameter<double>("max_step", 0.20);
    max_spread_ = declare_parameter<double>("max_spread", 0.20);
    hop_ = declare_parameter<double>("hop", 0.15);
    seed_r_ = declare_parameter<double>("seed_r", 1.5);
    h_stand_ = declare_parameter<double>("h_stand", 0.31);        // 몸체 원점(IMU)~바닥 (10-30 실측)
    seed_tol_ = declare_parameter<double>("seed_tol", 0.08);
    seed_slope_ = declare_parameter<double>("seed_slope", 0.05);
    peak_h_ = declare_parameter<double>("peak_h", 0.10);
    ring_in_ = declare_parameter<double>("ring_in", 0.15);
    ring_out_ = declare_parameter<double>("ring_out", 0.30);
    min_ring_ = declare_parameter<int>("min_ring", 6);
    // D435i (11-04 calib_depth.py, 10-30 녹화본): 광학 → FAST-LIO body
    fx_ = declare_parameter<double>("fx", 431.08); fy_ = declare_parameter<double>("fy", 431.08);
    cx_ = declare_parameter<double>("cx", 422.28); cy_ = declare_parameter<double>("cy", 240.89);
    T_bc_ = declare_parameter<std::vector<double>>("T_body_cam", {0.00198, -0.55953, 0.82881, 0.46644,
                                                                  -0.9993, -0.03209, -0.01927, 0.00958,
                                                                  0.03738, -0.82819, -0.5592, 0.11276});
    depth_every_ = declare_parameter<int>("depth_every", 3);        // 30 Hz → 10 Hz
    depth_stride_ = declare_parameter<int>("depth_stride", 4);
    depth_min_ = declare_parameter<double>("depth_min", 0.30);
    depth_max_ = declare_parameter<double>("depth_max", 2.5);
    depth_max_age_ = declare_parameter<double>("depth_max_age", 0.25);
    stats_period_ = declare_parameter<double>("stats_period", 5.0);
    // 깊이 점을 플래너로 어떻게 넘기나 (11-04 eval_veto3 갈래): all=통과면 아닌 점 전부 / none=격자에만 / structure=세로·솟음 칸 점만 /
    //                                                        ground=통과면 높이(0.3 m 안 최소)보다 depth_ground_h 넘게 솟은 점만
    depth_mode_ = declare_parameter<std::string>("depth_mode", "none");   // 10-30: 플래너로 보내면 헛장애물 5.3→10~12% — 격자에만
    depth_ground_h_ = declare_parameter<double>("depth_ground_h", 0.10);
    // 낭떠러지 (11-04, ramp/cliff): 통과면 가장자리 바깥이 cliff_drop 넘게 낮으면 가장자리에 장애물 점. 계단 한 단(0.17~0.20)은 통과
    cliff_enable_ = declare_parameter<bool>("cliff_enable", true);
    cliff_drop_ = declare_parameter<double>("cliff_drop", 0.22);
    cliff_reach_ = declare_parameter<double>("cliff_reach", 0.30);
    cliff_rmin_ = declare_parameter<double>("cliff_rmin", 0.05);
    cliff_min_low_ = declare_parameter<int>("cliff_min_low", 1);
    cliff_dz_ = declare_parameter<double>("cliff_dz", 0.30);          // 점 높이 = 로봇 z + 0.30 (ego 규칙 장애물 범위 안)
    cliff_band_ = declare_parameter<double>("cliff_band", 0.0);       // >0 이면 가장자리 높이가 발밑 평면 ±band 안일 때만. 0.20: X자 헛장애물 5.5→3.9% 이지만 단 위 뚫림 0.30→0.50% (11-04 eval_cliff) → 기본 끔

    n_ = static_cast<int>(std::lround(2 * half_ / cell_));
    hi_.resize(n_ * n_); lo_.resize(n_ * n_); cnt_.resize(n_ * n_); pk_.resize(n_ * n_);
    bad_.resize(n_ * n_); cand_.resize(n_ * n_); trav_.resize(n_ * n_);
    const int ri = static_cast<int>(std::lround(ring_in_ / cell_)), ro = static_cast<int>(std::lround(ring_out_ / cell_));
    for (int i = -ro; i <= ro; ++i)
      for (int j = -ro; j <= ro; ++j) {
        const double r = std::hypot(i, j);
        if (r >= ri && r <= ro) ring_.push_back({i, j});
      }
    const double hc = hop_ / cell_;
    const int h = static_cast<int>(std::floor(hc + 1e-6));   // 0.15/0.05=2.9999… → 축 3칸 이웃 누락 방지 (11-04 검토)
    for (int i = -h; i <= h; ++i)
      for (int j = -h; j <= h; ++j)
        if ((i || j) && i * i + j * j <= hc * hc + 1e-6) hop_offs_.push_back({i, j});
    cliff_offs_ = cliff_offsets(cell_, cliff_rmin_, cliff_reach_);

    auto sensor = rclcpp::SensorDataQoS();
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, sensor,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(m); });
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, sensor,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { on_cloud(m); });
    if (!l1_topic_.empty())
      sub_l1_ = create_subscription<sensor_msgs::msg::PointCloud2>(l1_topic_, sensor,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { on_l1(m); });
    if (!depth_topic_.empty())
      sub_depth_ = create_subscription<sensor_msgs::msg::Image>(depth_topic_, sensor,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_depth(m); });
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic_, rclcpp::QoS(10));
    pub_trav_ = create_publisher<sensor_msgs::msg::PointCloud2>("/terrain_veto/traversable", rclcpp::QoS(2));
    pub_cliff_ = create_publisher<sensor_msgs::msg::PointCloud2>("/terrain_veto/cliff", rclcpp::QoS(2));
    RCLCPP_INFO(get_logger(), "terrain_veto: %s + %s → %s  격자 %d×%d (%.2f m)  max_step %.2f  깊이 %s (플래너로: %s)  L1 %s",
                cloud_topic_.c_str(), odom_topic_.c_str(), out_topic_.c_str(), n_, n_, cell_, max_step_,
                depth_topic_.empty() ? "끔" : depth_topic_.c_str(), depth_mode_.c_str(), l1_topic_.empty() ? "끔" : l1_topic_.c_str());
    RCLCPP_INFO(get_logger(), "낭떠러지: %s  drop %.2f  reach %.2f (rmin %.2f, 오프셋 %zu)  min_low %ld  dz %.2f  band %.2f",
                cliff_enable_ ? "켬" : "끔", cliff_drop_, cliff_reach_, cliff_rmin_, cliff_offs_.size(), static_cast<long>(cliff_min_low_), cliff_dz_, cliff_band_);
  }

private:
  double tnow() { return now().seconds(); }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & m) {
    const auto & p = m->pose.pose.position; const auto & q = m->pose.pose.orientation;
    std::lock_guard<std::mutex> lk(mu_);
    odom_.push_back({tnow(), p.x, p.y, p.z, q.x, q.y, q.z, q.w});
    while (odom_.size() > 2 && odom_.back().t - odom_.front().t > 3.0) odom_.pop_front();
  }

  bool pose_at(double t, Pose & out) {   // mu_ 잡은 상태에서
    if (odom_.empty()) return false;
    if (t <= odom_.front().t) { out = odom_.front(); return true; }
    if (t >= odom_.back().t) { out = odom_.back(); return true; }
    auto it = std::lower_bound(odom_.begin(), odom_.end(), t, [](const Pose & a, double v) { return a.t < v; });
    const Pose & b = *it; const Pose & a = *(it - 1);
    const double s = (t - a.t) / std::max(b.t - a.t, 1e-6);
    out.t = t; out.x = a.x + s * (b.x - a.x); out.y = a.y + s * (b.y - a.y); out.z = a.z + s * (b.z - a.z);
    double sg = (a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw) < 0 ? -1.0 : 1.0;   // nlerp
    out.qx = a.qx + s * (sg * b.qx - a.qx); out.qy = a.qy + s * (sg * b.qy - a.qy);
    out.qz = a.qz + s * (sg * b.qz - a.qz); out.qw = a.qw + s * (sg * b.qw - a.qw);
    const double nn = std::sqrt(out.qx * out.qx + out.qy * out.qy + out.qz * out.qz + out.qw * out.qw);
    out.qx /= nn; out.qy /= nn; out.qz /= nn; out.qw /= nn;
    return true;
  }

  void on_depth(const sensor_msgs::msg::Image::ConstSharedPtr & m) {
    if (++depth_n_ % std::max(1, static_cast<int>(depth_every_))) return;
    if (m->encoding != "16UC1" && m->encoding != "mono16") return;
    const double recv = tnow(), st = rclcpp::Time(m->header.stamp).seconds();
    // 카메라 stamp 는 로봇 PC 시계 → (수신 − stamp) 최근 10초 최솟값으로 NUC 시각에 맞춘다
    off_.push_back({recv, recv - st});
    while (off_.size() > 1 && recv - off_.front().first > 10.0) off_.pop_front();
    double off = std::numeric_limits<double>::infinity();
    for (const auto & o : off_) off = std::min(off, o.second);
    const double tcap = st + off;
    Pose P;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!pose_at(tcap, P)) return;
    }
    double Rwb[9]; quat_to_R(P.qx, P.qy, P.qz, P.qw, Rwb);
    const double * T = T_bc_.data();
    // 광학 → 세계 한 번에: Rwc = Rwb·Rbc, twc = Rwb·tbc + twb
    double Rwc[9], twc[3];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) Rwc[i * 3 + j] = Rwb[i * 3] * T[j] + Rwb[i * 3 + 1] * T[4 + j] + Rwb[i * 3 + 2] * T[8 + j];
      twc[i] = Rwb[i * 3] * T[3] + Rwb[i * 3 + 1] * T[7] + Rwb[i * 3 + 2] * T[11] + (i == 0 ? P.x : i == 1 ? P.y : P.z);
    }
    Frame f; f.t = recv;
    const int W = m->width, H = m->height, s = std::max(1, static_cast<int>(depth_stride_));
    f.pts.reserve((W / s) * (H / s));
    for (int v = 0; v < H; v += s) {
      const uint16_t * row = reinterpret_cast<const uint16_t *>(m->data.data() + static_cast<size_t>(v) * m->step);
      for (int u = 0; u < W; u += s) {
        const double d = row[u] * 1e-3;
        if (d < depth_min_ || d > depth_max_) continue;
        const double px = (u - cx_) * d / fx_, py = (v - cy_) * d / fy_, pz = d;
        f.pts.push_back({static_cast<float>(Rwc[0] * px + Rwc[1] * py + Rwc[2] * pz + twc[0]),
                         static_cast<float>(Rwc[3] * px + Rwc[4] * py + Rwc[5] * pz + twc[1]),
                         static_cast<float>(Rwc[6] * px + Rwc[7] * py + Rwc[8] * pz + twc[2])});
      }
    }
    std::lock_guard<std::mutex> lk(mu_);
    dbuf_.push_back(std::move(f));
    while (!dbuf_.empty() && recv - dbuf_.front().t > acc_time_) dbuf_.pop_front();
    ++n_depth_ok_;
  }

  void on_l1(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & m) {   // 몸체 좌표 → 최신 자세로 세계
    std::vector<Pt> b;
    if (!read_xyz(*m, b)) return;
    const double t = tnow();
    std::lock_guard<std::mutex> lk(mu_);
    Pose P;
    if (!pose_at(t, P)) return;
    double R[9]; quat_to_R(P.qx, P.qy, P.qz, P.qw, R);
    Frame f; f.t = t; f.pts.reserve(b.size());
    for (const auto & p : b) {
      if (p.x * p.x + p.y * p.y + p.z * p.z < 0.04) continue;
      f.pts.push_back({static_cast<float>(R[0] * p.x + R[1] * p.y + R[2] * p.z + P.x),
                       static_cast<float>(R[3] * p.x + R[4] * p.y + R[5] * p.z + P.y),
                       static_cast<float>(R[6] * p.x + R[7] * p.y + R[8] * p.z + P.z)});
    }
    lbuf_.push_back(std::move(f));
    while (!lbuf_.empty() && t - lbuf_.front().t > acc_time_) lbuf_.pop_front();
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & m) {
    const auto t_start = std::chrono::steady_clock::now();
    const double t = tnow();
    Frame cur; cur.t = t;
    if (!read_xyz(*m, cur.pts)) return;
    std::lock_guard<std::mutex> lk(mu_);
    Pose P;
    if (!pose_at(t, P)) { pub_->publish(*m); return; }           // 자세 없으면 그대로 통과
    xbuf_.push_back(cur);
    while (!xbuf_.empty() && t - xbuf_.front().t > acc_time_) xbuf_.pop_front();
    while (!dbuf_.empty() && t - dbuf_.front().t > acc_time_) dbuf_.pop_front();
    while (!lbuf_.empty() && t - lbuf_.front().t > acc_time_) lbuf_.pop_front();

    // ① 높이 격자
    const double ox = P.x - half_, oy = P.y - half_, zmax = P.z + max_h_;
    std::fill(hi_.begin(), hi_.end(), -std::numeric_limits<float>::infinity());
    std::fill(lo_.begin(), lo_.end(), std::numeric_limits<float>::infinity());
    std::fill(cnt_.begin(), cnt_.end(), 0);
    auto add = [&](const std::vector<Pt> & v) {
      for (const auto & p : v) {
        if (p.z > zmax) continue;
        const int ix = static_cast<int>(std::floor((p.x - ox) / cell_)), iy = static_cast<int>(std::floor((p.y - oy) / cell_));
        if (ix < 0 || iy < 0 || ix >= n_ || iy >= n_) continue;
        const int k = ix * n_ + iy;
        hi_[k] = std::max(hi_[k], p.z); lo_[k] = std::min(lo_[k], p.z); ++cnt_[k];
      }
    };
    for (const auto & f : xbuf_) add(f.pts);
    for (const auto & f : dbuf_) add(f.pts);
    for (const auto & f : lbuf_) add(f.pts);

    // ② 솟음(고리 80% 높이 대비), 세로 구조 → 제외(+옆 1칸)
    std::vector<float> ring; ring.reserve(ring_.size());
    std::fill(bad_.begin(), bad_.end(), 0);
    for (int i = 0; i < n_; ++i)
      for (int j = 0; j < n_; ++j) {
        const int k = i * n_ + j;
        if (!cnt_[k]) continue;
        bool b = (hi_[k] - lo_[k]) > max_spread_;
        if (!b) {
          ring.clear();
          for (const auto & o : ring_) {
            const int a = i + o[0], c = j + o[1];
            if (a < 0 || c < 0 || a >= n_ || c >= n_) continue;
            const int kk = a * n_ + c;
            if (cnt_[kk]) ring.push_back(hi_[kk]);
          }
          if (static_cast<int>(ring.size()) >= min_ring_) {
            const double pos = 0.8 * (ring.size() - 1);
            const size_t l = static_cast<size_t>(std::floor(pos));
            std::nth_element(ring.begin(), ring.begin() + l, ring.end());
            double q = ring[l];
            if (l + 1 < ring.size()) q += (pos - l) * (*std::min_element(ring.begin() + l + 1, ring.end()) - ring[l]);
            b = (hi_[k] - q) > peak_h_;
          }
        }
        bad_[k] = b;
      }
    for (int i = 0; i < n_; ++i)
      for (int j = 0; j < n_; ++j) {
        const int k = i * n_ + j;
        bool b = bad_[k] & 1;
        if (!b) {
          if (i > 0 && (bad_[k - n_] & 1)) b = true; else if (i + 1 < n_ && (bad_[k + n_] & 1)) b = true;
          else if (j > 0 && (bad_[k - 1] & 1)) b = true; else if (j + 1 < n_ && (bad_[k + 1] & 1)) b = true;
        }
        cand_[k] = cnt_[k] && !b;
      }

    // ③ 발밑에서 이어진 통과 면 (BFS). 씨앗 = 발밑 지면 평면(법선 = 몸체 z축) 근처 칸 — XT16 은 1.75 m 안 바닥을 못 봐서
    double Rb[9]; quat_to_R(P.qx, P.qy, P.qz, P.qw, Rb);
    const double nx = Rb[2], ny = Rb[5], nz = std::max(Rb[8], 0.5);
    const double fx = P.x - h_stand_ * nx, fy = P.y - h_stand_ * ny, fz = P.z - h_stand_ * nz;
    std::fill(trav_.begin(), trav_.end(), 0);
    std::vector<int> q; q.reserve(4096);
    const int ci = static_cast<int>(std::floor((P.x - ox) / cell_)), cj = static_cast<int>(std::floor((P.y - oy) / cell_));
    const int sr = static_cast<int>(std::ceil(seed_r_ / cell_));
    for (int i = std::max(0, ci - sr); i <= std::min(n_ - 1, ci + sr); ++i)
      for (int j = std::max(0, cj - sr); j <= std::min(n_ - 1, cj + sr); ++j) {
        const int k = i * n_ + j;
        const double cxw = ox + (i + 0.5) * cell_, cyw = oy + (j + 0.5) * cell_;
        const double dd = std::hypot(cxw - P.x, cyw - P.y);
        const double zp = fz - (nx * (cxw - fx) + ny * (cyw - fy)) / nz;
        if (cand_[k] && dd <= seed_r_ && std::fabs(hi_[k] - zp) <= seed_tol_ + seed_slope_ * dd) { trav_[k] = 1; q.push_back(k); }
      }
    const int n_seed = static_cast<int>(q.size());
    for (size_t h = 0; h < q.size(); ++h) {
      const int k = q[h], i = k / n_, j = k % n_;
      for (const auto & o : hop_offs_) {
        const int a = i + o[0], c = j + o[1];
        if (a < 0 || c < 0 || a >= n_ || c >= n_) continue;
        const int kk = a * n_ + c;
        if (trav_[kk] || !cand_[kk]) continue;
        if (std::fabs(hi_[kk] - hi_[k]) > max_step_) continue;
        trav_[kk] = 1; q.push_back(kk);
      }
    }

    // ④ 이번 스캔 + 최신 깊이 프레임 중 통과 면이 아닌 점
    std::vector<Pt> out; out.reserve(cur.pts.size() + 30000);
    size_t n_in = 0, n_rm = 0;
    auto keep = [&](const std::vector<Pt> & v) {
      for (const auto & p : v) {
        ++n_in;
        const int ix = static_cast<int>(std::floor((p.x - ox) / cell_)), iy = static_cast<int>(std::floor((p.y - oy) / cell_));
        if (ix >= 0 && iy >= 0 && ix < n_ && iy < n_ && trav_[ix * n_ + iy]) { ++n_rm; continue; }
        out.push_back(p);
      }
    };
    keep(cur.pts);
    // ⑤ 낭떠러지 가장자리 → 장애물 점 (가장자리 칸 중심, z = 로봇 z + cliff_dz)
    CliffStats cs;
    std::vector<Pt> cliff_pts;
    if (cliff_enable_) {
      const double plane6[6] = {fx, fy, fz, nx, ny, nz};
      cs = find_cliff(n_, hi_, cnt_, trav_, cliff_offs_, cliff_drop_, static_cast<int>(cliff_min_low_), cliff_, nullptr, nullptr,
                      cliff_band_, plane6, ox, oy, cell_);
      const float cz = static_cast<float>(P.z + cliff_dz_);
      for (int k = 0; k < n_ * n_; ++k)
        if (cliff_[k]) cliff_pts.push_back({static_cast<float>(ox + (k / n_ + 0.5) * cell_), static_cast<float>(oy + (k % n_ + 0.5) * cell_), cz});
      out.insert(out.end(), cliff_pts.begin(), cliff_pts.end());
    }
    bool used_depth = false;
    if (!dbuf_.empty() && t - dbuf_.back().t <= depth_max_age_ && depth_mode_ != "none") {
      used_depth = true;
      if (depth_mode_ == "all") {
        keep(dbuf_.back().pts);
      } else {
        if (depth_mode_ == "ground") {   // 통과면 높이를 0.3 m 안 최솟값으로 퍼뜨림
          const int R = static_cast<int>(std::lround(0.30 / cell_));
          std::fill(pk_.begin(), pk_.end(), std::numeric_limits<float>::infinity());
          for (int i = 0; i < n_; ++i)
            for (int j = 0; j < n_; ++j) {
              const int k = i * n_ + j;
              if (!trav_[k]) continue;
              for (int a = std::max(0, i - R); a <= std::min(n_ - 1, i + R); ++a)
                for (int c = std::max(0, j - R); c <= std::min(n_ - 1, j + R); ++c)
                  if ((a - i) * (a - i) + (c - j) * (c - j) <= R * R) pk_[a * n_ + c] = std::min(pk_[a * n_ + c], hi_[k]);
            }
        }
        for (const auto & p : dbuf_.back().pts) {
          ++n_in;
          const int ix = static_cast<int>(std::floor((p.x - ox) / cell_)), iy = static_cast<int>(std::floor((p.y - oy) / cell_));
          if (ix < 0 || iy < 0 || ix >= n_ || iy >= n_) { ++n_rm; continue; }
          const int k = ix * n_ + iy;
          bool ok = !trav_[k];
          if (depth_mode_ == "structure") ok = ok && (bad_[k] & 1);
          else if (depth_mode_ == "ground") ok = ok && std::isfinite(pk_[k]) && (p.z - pk_[k] >= depth_ground_h_);
          if (ok) out.push_back(p); else ++n_rm;
        }
      }
    }
    publish(out, m->header);
    if (pub_trav_->get_subscription_count() > 0 && (++n_cloud_ % 5 == 0)) {
      std::vector<Pt> tv;
      for (int k = 0; k < n_ * n_; ++k)
        if (trav_[k]) tv.push_back({static_cast<float>(ox + (k / n_ + 0.5) * cell_), static_cast<float>(oy + (k % n_ + 0.5) * cell_), hi_[k]});
      publish_to(pub_trav_, tv, m->header);
    }
    if (cliff_enable_ && pub_cliff_->get_subscription_count() > 0) publish_to(pub_cliff_, cliff_pts, m->header);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
    st_ms_ = std::max(st_ms_, ms); st_sum_ms_ += ms; ++st_n_; st_rm_ += n_rm; st_in_ += n_in; st_seed_ += n_seed;
    st_depth_ += used_depth;
    st_edge_ += cs.edge; st_cliff_ += cs.cliff; st_empty_ += cs.empty; st_cliff_max_ = std::max(st_cliff_max_, cs.cliff);
    if (t - st_t_ > stats_period_) {
      int ntrav = 0; for (auto v : trav_) ntrav += v;
      const double nn = std::max(1, st_n_);
      RCLCPP_INFO(get_logger(), "%.0f초: %d회 평균 %.1f ms 최대 %.1f ms | 지운 점 %.1f%% | 통과면 %d칸 씨앗 평균 %.0f | 깊이 포함 %d/%d (누적 %d프레임)"
                  " | 낭떠러지 가장자리 평균 %.1f칸(최대 %d) / 가장자리 %.0f칸 / 빈 가장자리 %.0f칸",
                  stats_period_, st_n_, st_sum_ms_ / std::max(1, st_n_), st_ms_, 100.0 * st_rm_ / std::max<size_t>(1, st_in_),
                  ntrav, static_cast<double>(st_seed_) / std::max(1, st_n_), st_depth_, st_n_, n_depth_ok_,
                  st_cliff_ / nn, st_cliff_max_, st_edge_ / nn, st_empty_ / nn);
      st_t_ = t; st_ms_ = 0; st_sum_ms_ = 0; st_n_ = 0; st_rm_ = 0; st_in_ = 0; st_seed_ = 0; st_depth_ = 0;
      st_edge_ = 0; st_cliff_ = 0; st_empty_ = 0; st_cliff_max_ = 0;
    }
  }

  void publish_to(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub, const std::vector<Pt> & v,
                  const std_msgs::msg::Header & h) {
    sensor_msgs::msg::PointCloud2 c; c.header = h; c.height = 1; c.width = static_cast<uint32_t>(v.size());
    c.is_bigendian = false; c.is_dense = true; c.point_step = 12; c.row_step = 12 * c.width;
    const char * nm[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField f; f.name = nm[i]; f.offset = 4 * i; f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
      c.fields.push_back(f);
    }
    c.data.resize(c.row_step);
    if (!v.empty()) std::memcpy(c.data.data(), v.data(), c.row_step);
    pub->publish(std::move(c));
  }
  void publish(const std::vector<Pt> & v, const std_msgs::msg::Header & h) { publish_to(pub_, v, h); }

  std::string cloud_topic_, odom_topic_, out_topic_, depth_topic_, l1_topic_, depth_mode_;
  double depth_ground_h_;
  bool cliff_enable_;
  double cliff_drop_, cliff_reach_, cliff_rmin_, cliff_dz_, cliff_band_;
  int64_t cliff_min_low_;
  std::vector<std::array<int, 2>> cliff_offs_;
  std::vector<uint8_t> cliff_;
  int64_t st_edge_ = 0, st_cliff_ = 0, st_empty_ = 0; int st_cliff_max_ = 0;
  double acc_time_, cell_, half_, max_h_, max_step_, max_spread_, hop_, seed_r_, h_stand_, seed_tol_, seed_slope_, peak_h_, ring_in_, ring_out_;
  int64_t min_ring_, depth_every_, depth_stride_;
  double fx_, fy_, cx_, cy_, depth_min_, depth_max_, depth_max_age_, stats_period_;
  std::vector<double> T_bc_;
  int n_;
  std::vector<float> hi_, lo_, pk_;
  std::vector<int> cnt_;
  std::vector<uint8_t> bad_, cand_, trav_;
  std::vector<std::array<int, 2>> ring_, hop_offs_;
  std::deque<Pose> odom_;
  std::deque<Frame> xbuf_, dbuf_, lbuf_;
  std::deque<std::pair<double, double>> off_;
  std::mutex mu_;
  int64_t depth_n_ = 0;
  int n_depth_ok_ = 0, n_cloud_ = 0;
  double st_t_ = 0, st_ms_ = 0, st_sum_ms_ = 0; int st_n_ = 0, st_depth_ = 0; size_t st_rm_ = 0, st_in_ = 0; int64_t st_seed_ = 0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_, sub_l1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_, pub_trav_, pub_cliff_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainVeto>());
  rclcpp::shutdown();
  return 0;
}
