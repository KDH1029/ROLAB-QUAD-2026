#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace go2_bridge
{

/**
 * @brief Forward Kinematics and Ground Plane Attitude Estimator for Unitree Go2.
 *
 * Go2 Link Parameters (URDF-derived):
 * - Hip offsets relative to body center:
 *     FR: (+0.1934, -0.0465, 0.0)
 *     FL: (+0.1934, +0.0465, 0.0)
 *     RR: (-0.1934, -0.0465, 0.0)
 *     RL: (-0.1934, +0.0465, 0.0)
 * - Thigh lateral offset (d_y):
 *     Right legs (FR=0, RR=2): -0.0955 m
 *     Left legs  (FL=1, RL=3): +0.0955 m
 * - Thigh link length (L1): 0.213 m
 * - Calf link length  (L2): 0.213 m
 *
 * Motor index order in LowState:
 *   FR: 0 (hip/roll),  1 (thigh/pitch), 2 (calf/pitch)
 *   FL: 3 (hip/roll),  4 (thigh/pitch), 5 (calf/pitch)
 *   RR: 6 (hip/roll),  7 (thigh/pitch), 8 (calf/pitch)
 *   RL: 9 (hip/roll), 10 (thigh/pitch), 11 (calf/pitch)
 */
class Go2Kinematics
{
public:
  struct Vec3
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct KinematicsResult
  {
    std::array<Vec3, 4> foot_pos;     // Foot position in body frame
    std::array<bool, 4> is_stance;    // Stance flag based on contact force
    int stance_count{0};
    double roll_kin{0.0};             // Kinematic roll (rad, body relative to ground)
    double pitch_kin{0.0};            // Kinematic pitch (rad, body relative to ground)
    double height_kin{0.28};          // Distance from body to ground (m)
    bool valid{false};
  };

  Go2Kinematics()
  {
    // Leg base positions relative to body center [FR, FL, RR, RL]
    hip_base_[0] = { 0.1934, -0.0465, 0.0}; // FR
    hip_base_[1] = { 0.1934,  0.0465, 0.0}; // FL
    hip_base_[2] = {-0.1934, -0.0465, 0.0}; // RR
    hip_base_[3] = {-0.1934,  0.0465, 0.0}; // RL

    thigh_offset_[0] = -0.0955; // FR
    thigh_offset_[1] =  0.0955; // FL
    thigh_offset_[2] = -0.0955; // RR
    thigh_offset_[3] =  0.0955; // RL
  }

  /**
   * @brief Compute forward kinematics for a single leg.
   * @param leg_idx 0: FR, 1: FL, 2: RR, 3: RL
   * @param q1 Hip abduction/adduction angle (rad)
   * @param q2 Thigh angle (rad)
   * @param q3 Calf angle (rad)
   * @return Vec3 Foot position in base_link coordinates (m)
   */
  Vec3 forwardKinematics(int leg_idx, double q1, double q2, double q3) const
  {
    const double d_y = thigh_offset_[leg_idx];
    const double s1 = std::sin(q1);
    const double c1 = std::cos(q1);
    const double s2 = std::sin(q2);
    const double c2 = std::cos(q2);
    const double s23 = std::sin(q2 + q3);
    const double c23 = std::cos(q2 + q3);

    const double x_h = -l1_ * s2 - l2_ * s23;
    const double leg_reach_y = l1_ * c2 + l2_ * c23;
    const double y_h = d_y * c1 + leg_reach_y * s1;
    const double z_h = d_y * s1 - leg_reach_y * c1;

    Vec3 foot;
    foot.x = hip_base_[leg_idx].x + x_h;
    foot.y = hip_base_[leg_idx].y + y_h;
    foot.z = hip_base_[leg_idx].z + z_h;
    return foot;
  }

  /**
   * @brief Estimate robot ground plane orientation and height from joints and foot contact.
   *
   * Solves both 3D least-squares plane fitting (when >=3 non-collinear stance feet exist)
   * and diagonal/orthogonal vector projection (when 2 diagonal stance feet exist during trotting).
   *
   * Sign Convention (ROS 2 Right-Handed Body Frame: +X forward, +Y left, +Z up):
   * - Ground plane in body frame: z_i = z_0 + tan(pitch)*x_i - tan(roll)*y_i
   * - Pitch > 0 (nose down): front feet are higher in body frame than rear feet (z_front > z_rear)
   * - Roll  > 0 (right side down): left feet are lower in body frame than right feet (z_right > z_left)
   *
   * @param motor_q Array of 12 joint positions
   * @param foot_force Array of 4 contact force readings
   * @param contact_thresh Threshold in raw units to declare stance (default 15)
   */
  KinematicsResult compute(
    const std::array<double, 12> & motor_q,
    const std::array<int16_t, 4> & foot_force,
    int16_t contact_thresh = 15) const
  {
    KinematicsResult res;
    res.stance_count = 0;

    bool all_zero_q = true;
    for (int i = 0; i < 12; ++i) {
      if (std::abs(motor_q[i]) > 1e-5) {
        all_zero_q = false;
        break;
      }
    }
    if (all_zero_q) {
      // Uninitialized or mock message with zero joint angles
      res.valid = false;
      return res;
    }

    for (int i = 0; i < 4; ++i) {
      const double q1 = motor_q[i * 3 + 0];
      const double q2 = motor_q[i * 3 + 1];
      const double q3 = motor_q[i * 3 + 2];
      res.foot_pos[i] = forwardKinematics(i, q1, q2, q3);

      res.is_stance[i] = (foot_force[i] > contact_thresh);
      if (res.is_stance[i]) {
        res.stance_count++;
      }
    }

    // Select stance feet (fallback to all 4 feet if < 2 feet report contact)
    std::vector<int> stance_indices;
    if (res.stance_count >= 2) {
      for (int i = 0; i < 4; ++i) {
        if (res.is_stance[i]) {
          stance_indices.push_back(i);
        }
      }
    } else {
      stance_indices = {0, 1, 2, 3};
    }

    // 1. Compute average body height relative to stance feet
    double x_mean = 0.0, y_mean = 0.0, z_mean = 0.0;
    const double n_pts = static_cast<double>(stance_indices.size());
    for (int idx : stance_indices) {
      x_mean += res.foot_pos[idx].x;
      y_mean += res.foot_pos[idx].y;
      z_mean += res.foot_pos[idx].z;
    }
    x_mean /= n_pts;
    y_mean /= n_pts;
    z_mean /= n_pts;
    res.height_kin = -z_mean;

    // 2. If >= 3 stance feet, solve full 2D linear least-squares plane:
    //    dz = a * dx + b * dy   where pitch = atan(a), roll = atan(-b)
    bool plane_solved = false;
    if (stance_indices.size() >= 3) {
      double s_xx = 0.0, s_yy = 0.0, s_xy = 0.0, s_xz = 0.0, s_yz = 0.0;
      for (int idx : stance_indices) {
        const double dx = res.foot_pos[idx].x - x_mean;
        const double dy = res.foot_pos[idx].y - y_mean;
        const double dz = res.foot_pos[idx].z - z_mean;
        s_xx += dx * dx;
        s_yy += dy * dy;
        s_xy += dx * dy;
        s_xz += dx * dz;
        s_yz += dy * dz;
      }
      const double det = s_xx * s_yy - s_xy * s_xy;
      if (std::abs(det) > 1e-5) {
        const double a = (s_yy * s_xz - s_xy * s_yz) / det;
        const double b = (s_xx * s_yz - s_xy * s_xz) / det;
        res.pitch_kin = std::atan(a);
        res.roll_kin = std::atan(-b);
        plane_solved = true;
      }
    }

    // 3. Fallback for 2 stance feet (e.g. diagonal trot pair [FR, RL] or [FL, RR]):
    //    Compute longitudinal pitch from front vs rear stance feet, and lateral roll from left vs right
    if (!plane_solved) {
      double z_front = 0.0, x_front = 0.0; int n_front = 0;
      double z_rear = 0.0,  x_rear = 0.0;  int n_rear = 0;
      double z_left = 0.0,  y_left = 0.0;  int n_left = 0;
      double z_right = 0.0, y_right = 0.0; int n_right = 0;

      for (int idx : stance_indices) {
        const auto & f = res.foot_pos[idx];
        if (idx == 0 || idx == 1) {
          z_front += f.z; x_front += f.x; n_front++;
        } else {
          z_rear += f.z; x_rear += f.x; n_rear++;
        }
        if (idx == 1 || idx == 3) {
          z_left += f.z; y_left += f.y; n_left++;
        } else {
          z_right += f.z; y_right += f.y; n_right++;
        }
      }

      if (n_front > 0 && n_rear > 0) {
        x_front /= n_front; z_front /= n_front;
        x_rear /= n_rear;   z_rear /= n_rear;
        const double dx = x_front - x_rear;
        if (std::abs(dx) > 0.1) {
          res.pitch_kin = std::atan2(z_front - z_rear, dx);
        }
      }

      if (n_left > 0 && n_right > 0) {
        y_left /= n_left;   z_left /= n_left;
        y_right /= n_right; z_right /= n_right;
        const double dy = y_left - y_right;
        if (std::abs(dy) > 0.1) {
          // Right-handed roll: roll > 0 means right side drops -> z_right > z_left in body frame
          res.roll_kin = std::atan2(z_right - z_left, dy);
        }
      }
    }

    res.valid = true;
    return res;
  }

private:
  std::array<Vec3, 4> hip_base_;
  std::array<double, 4> thigh_offset_;
  const double l1_{0.213};
  const double l2_{0.213};
};

}  // namespace go2_bridge
