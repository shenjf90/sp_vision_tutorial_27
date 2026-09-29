#ifndef TASK_COMMON_HPP
#define TASK_COMMON_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace tutorial
{
enum class Task { detection, vehicle, outpost };

inline bool relevant(const auto_aim::Armor & armor, Task task)
{
  return armor.name != auto_aim::not_armor &&
         (task == Task::outpost ? armor.name == auto_aim::outpost
                               : task == Task::detection || armor.name != auto_aim::outpost);
}

inline void aim_at(io::Gimbal & gimbal, const Eigen::Vector3d & point)
{
  const auto state = gimbal.state();
  const double distance = std::hypot(point.x(), point.y());
  if (!point.allFinite() || distance < 0.1 || !std::isfinite(state.bullet_speed) ||
      state.bullet_speed < 1.0f) {
    gimbal.send(false, false, 0, 0);
    return;
  }
  tools::Trajectory trajectory(state.bullet_speed, distance, point.z());
  if (trajectory.unsolvable || !std::isfinite(trajectory.pitch)) {
    gimbal.send(false, false, 0, 0);
    return;
  }
  gimbal.send(true, false, static_cast<float>(std::atan2(point.y(), point.x())),
              static_cast<float>(trajectory.pitch));
}

inline int run(Task task, const std::string & config_path)
{
  using clock = std::chrono::steady_clock;
  tools::Exiter exiter;
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  std::optional<auto_aim::Target> target;
  clock::time_point last_seen{};
  int frame = 0;

  while (!exiter.exit()) {
    cv::Mat img;
    clock::time_point t;
    if (!camera.try_read_for(img, t, std::chrono::milliseconds(100))) {
      gimbal.send(false, false, 0, 0);
      if (!camera.is_alive()) break;
      continue;
    }
    if (img.empty()) continue;

    solver.set_R_gimbal2world(gimbal.q(t));
    auto armors = yolo.detect(img, frame++);
    for (auto & armor : armors) {
      if (relevant(armor, task)) solver.solve(armor);
    }

    if (target && t - last_seen > std::chrono::milliseconds(300)) target.reset();
    if (target) target->predict(t);
    const auto expected = target ? target->armor_xyza_list() : std::vector<Eigen::Vector4d>{};
    auto best = armors.end();
    double best_distance = std::numeric_limits<double>::infinity();
    for (auto it = armors.begin(); it != armors.end(); ++it) {
      if (!relevant(*it, task) || !it->xyz_in_world.allFinite()) continue;
      if (target && it->name != target->name) continue;
      double distance = it->xyz_in_world.norm();
      if (target) {
        distance = std::numeric_limits<double>::infinity();
        for (const auto & plate : expected) {
          distance = std::min(distance, (it->xyz_in_world - plate.head<3>()).norm());
        }
        if (distance > 0.8) continue;
      }
      if (distance < best_distance) {
        best = it;
        best_distance = distance;
      }
    }

    if (task == Task::detection) {
      if (best != armors.end()) aim_at(gimbal, best->xyz_in_world);
      else gimbal.send(false, false, 0, 0);
      continue;
    }

    if (!target && best != armors.end()) {
      const bool outpost = task == Task::outpost;
      target.emplace(*best, t, outpost ? 0.2765 : 0.2, outpost ? 3 : 4);
      last_seen = t;
    } else if (target) {
      if (best != armors.end()) {
        target->update(*best);
        last_seen = t;
      }
      if (target->diverged()) target.reset();
    }

    if (!target || t - last_seen > std::chrono::milliseconds(100)) {
      gimbal.send(false, false, 0, 0);
      continue;
    }

    // 按弹丸飞行时间预测，并选离相机最近的装甲板。
    auto predicted = *target;
    const double speed = gimbal.state().bullet_speed;
    if (speed > 1.0) {
      const auto x = target->ekf_x();
      predicted.predict(std::clamp(std::hypot(x[0], x[2]) / speed, 0.0, 0.5));
    }
    const auto plates = predicted.armor_xyza_list();
    const auto nearest = std::min_element(plates.begin(), plates.end(),
      [](const auto & a, const auto & b) {
        return a.template head<3>().squaredNorm() < b.template head<3>().squaredNorm();
      });
    if (nearest != plates.end()) aim_at(gimbal, nearest->head<3>());
  }
  gimbal.send(false, false, 0, 0);
  return 0;
}
}  // namespace tutorial

#endif
