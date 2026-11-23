#include "isweep_planner/optimization/backend/trajectory_optimizer.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <Eigen/Dense>

namespace isweep_planner {

namespace {

Eigen::Matrix<double, 6, 1> SolveQuinticBoundarySystem(
    double duration, double p0, double v0, double a0, double p1, double v1,
    double a1) {
  const double t = std::max(1e-3, duration);
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;
  Eigen::Matrix<double, 6, 6> system;
  system << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, t,
      t2, t3, t4, t5, 0, 1, 2 * t, 3 * t2, 4 * t3, 5 * t4, 0, 0, 2, 6 * t,
      12 * t2, 20 * t3;
  Eigen::Matrix<double, 6, 1> rhs;
  rhs << p0, v0, a0, p1, v1, a1;
  return system.colPivHouseholderQr().solve(rhs);
}

PolyPiece ConditionPositionBoundary(const PolyPiece& previous,
                                    const PolyPiece& current) {
  PolyPiece conditioned = current;
  const double previous_t = previous.duration;
  const double current_t = current.duration;
  const Eigen::Vector2d p0 = previous.evaluate(previous_t);
  const Eigen::Vector2d v0 = previous.velocity(previous_t);
  const Eigen::Vector2d a0 = previous.acceleration(previous_t);
  const Eigen::Vector2d p1 = current.evaluate(current_t);
  const Eigen::Vector2d v1 = current.velocity(current_t);
  const Eigen::Vector2d a1 = current.acceleration(current_t);
  for (int axis = 0; axis < 2; ++axis) {
    const Eigen::Matrix<double, 6, 1> coeffs = SolveQuinticBoundarySystem(
        current_t, p0(axis), v0(axis), a0(axis), p1(axis), v1(axis), a1(axis));
    conditioned.coeffs.row(axis) = coeffs.transpose();
  }
  return conditioned;
}

PolyPiece RefitPositionPiece(const PolyPiece& source,
                             const Eigen::Vector2d& p0,
                             const Eigen::Vector2d& v0,
                             const Eigen::Vector2d& a0,
                             const Eigen::Vector2d& p1,
                             const Eigen::Vector2d& v1,
                             const Eigen::Vector2d& a1) {
  PolyPiece refit = source;
  for (int axis = 0; axis < 2; ++axis) {
    const Eigen::Matrix<double, 6, 1> coeffs = SolveQuinticBoundarySystem(
        source.duration, p0(axis), v0(axis), a0(axis), p1(axis), v1(axis),
        a1(axis));
    refit.coeffs.row(axis) = coeffs.transpose();
  }
  return refit;
}

Eigen::Vector2d SharedGeometricBoundaryVelocity(
    const PolyPiece& previous, const PolyPiece& current,
    const OptimizerParams& params) {
  constexpr double kMinimumChord = 1e-5;
  constexpr double kMinimumBisectorNorm = 1e-3;
  const Eigen::Vector2d boundary = previous.evaluate(previous.duration);
  const Eigen::Vector2d incoming = boundary - previous.evaluate(0.0);
  const Eigen::Vector2d outgoing =
      current.evaluate(current.duration) - boundary;
  const double incoming_length = incoming.norm();
  const double outgoing_length = outgoing.norm();
  if (incoming_length <= kMinimumChord || outgoing_length <= kMinimumChord) {
    return Eigen::Vector2d::Zero();
  }

  const Eigen::Vector2d incoming_direction = incoming / incoming_length;
  const Eigen::Vector2d outgoing_direction = outgoing / outgoing_length;
  Eigen::Vector2d tangent = incoming_direction + outgoing_direction;
  const double tangent_norm = tangent.norm();
  // A near reversal requires an explicit reversing maneuver; inventing a
  // bisector in that case would create a lateral loop.
  if (tangent_norm <= kMinimumBisectorNorm) {
    return Eigen::Vector2d::Zero();
  }
  tangent /= tangent_norm;

  const double incoming_chord_speed =
      incoming_length / std::max(1e-3, previous.duration);
  const double outgoing_chord_speed =
      outgoing_length / std::max(1e-3, current.duration);
  const double speed_scale =
      std::max(0.0, params.stitch_tangent_speed_scale);
  const double speed_ratio =
      std::max(0.0, params.stitch_max_tangent_speed_ratio);
  const double adjacent_speed =
      std::min(incoming_chord_speed, outgoing_chord_speed);
  const double shared_speed = std::min(
      speed_scale * adjacent_speed,
      speed_ratio * std::max(1e-3, params.max_vel));
  if (!std::isfinite(shared_speed) || shared_speed <= 1e-6) {
    return Eigen::Vector2d::Zero();
  }
  return shared_speed * tangent;
}

YawPolyPiece ConditionYawBoundary(const YawPolyPiece& previous,
                                  const YawPolyPiece& current) {
  YawPolyPiece conditioned = current;
  const double previous_t = previous.duration;
  const double current_t = current.duration;
  const double yaw0 = previous.evaluate(previous_t);
  const double yaw1 = yaw0 + normalizeAngle(current.evaluate(current_t) - yaw0);
  const Eigen::Matrix<double, 6, 1> coeffs = SolveQuinticBoundarySystem(
      current_t, yaw0, previous.velocity(previous_t),
      previous.acceleration(previous_t), yaw1, current.velocity(current_t),
      current.acceleration(current_t));
  conditioned.coeffs = coeffs.transpose();
  return conditioned;
}

bool IsTurnSupportPoint(const std::vector<Eigen::Vector2d>& positions,
                        int index, double maximum_heading_change) {
  if (index <= 0 || index + 1 >= static_cast<int>(positions.size())) {
    return false;
  }
  const Eigen::Vector2d incoming = positions[index] - positions[index - 1];
  const Eigen::Vector2d outgoing = positions[index + 1] - positions[index];
  if (incoming.norm() < 1e-8 || outgoing.norm() < 1e-8) return false;
  const double incoming_heading = std::atan2(incoming.y(), incoming.x());
  const double outgoing_heading = std::atan2(outgoing.y(), outgoing.x());
  return std::abs(normalizeAngle(outgoing_heading - incoming_heading)) >=
         maximum_heading_change;
}

}  // namespace

TrajectoryOptimizer::TrajectoryOptimizer() {}

void TrajectoryOptimizer::init(const GridMap& map, const SvsdfEvaluator& svsdf,
                               const OptimizerParams& params) {
  map_ = &map;
  svsdf_ = &svsdf;
  params_ = params;
}

std::vector<double> TrajectoryOptimizer::allocateTime(
    const std::vector<SE2State>& waypoints, double total_time) {
  const int piece_count = static_cast<int>(waypoints.size()) - 1;
  std::vector<double> time_demands(piece_count, 0.0);
  double total_demand = 0.0;
  const double max_vel = std::max(0.10, params_.max_vel);
  const double max_yaw_rate = std::max(0.10, params_.max_yaw_rate);
  for (int i = 0; i < piece_count; ++i) {
    const double dx = waypoints[i + 1].x - waypoints[i].x;
    const double dy = waypoints[i + 1].y - waypoints[i].y;
    const double translation_time = std::hypot(dx, dy) / max_vel;
    const double rotation_time =
        std::abs(normalizeAngle(waypoints[i + 1].yaw - waypoints[i].yaw)) /
        max_yaw_rate;
    // A corner can contain substantial rotation with almost no translation.
    // Distance-only allocation starved such a piece to 0.01 s and converted a
    // temporal failure into an unnecessary strict-SVSDF escalation.
    time_demands[i] = std::max(1e-6, std::max(translation_time, rotation_time));
    total_demand += time_demands[i];
  }
  std::vector<double> durations(piece_count, 0.01);
  if (piece_count <= 0 || total_demand <= 0.0) {
    return durations;
  }
  for (int i = 0; i < piece_count; ++i) {
    durations[i] = total_time * time_demands[i] / total_demand;
    if (durations[i] < 0.01) {
      durations[i] = 0.01;
    }
  }
  return durations;
}

void TrajectoryOptimizer::fitMincoQuintic(
    const std::vector<Eigen::Vector2d>& positions,
    const std::vector<double>& durations,
    const Eigen::Vector2d& v0, const Eigen::Vector2d& vf,
    const Eigen::Vector2d& a0, const Eigen::Vector2d& af,
    std::vector<PolyPiece>& pieces) {
  const int piece_count = static_cast<int>(positions.size()) - 1;
  if (piece_count <= 0) {
    pieces.clear();
    return;
  }

  std::vector<Eigen::Vector2d> velocities(piece_count + 1);
  std::vector<Eigen::Vector2d> accelerations(piece_count + 1);
  velocities[0] = v0;
  velocities[piece_count] = vf;
  accelerations[0] = a0;
  accelerations[piece_count] = af;

  for (int i = 1; i < piece_count; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-6) {
      dt_sum = 1e-6;
    }
    velocities[i] = (positions[i + 1] - positions[i - 1]) / dt_sum;
    const double vel_norm = velocities[i].norm();
    const double max_vel = std::max(1e-3, params_.max_vel);
    if (vel_norm > max_vel) {
      velocities[i] *= max_vel / vel_norm;
    }
  }

  for (int i = 1; i < piece_count; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-6) {
      dt_sum = 1e-6;
    }
    accelerations[i] =
        (velocities[std::min(i + 1, piece_count)] -
         velocities[std::max(i - 1, 0)]) /
        dt_sum;
    const double acc_norm = accelerations[i].norm();
    const double max_acc = std::max(1e-3, params_.max_acc);
    if (acc_norm > max_acc) {
      accelerations[i] *= max_acc / acc_norm;
    }
  }

  pieces.resize(piece_count);
  for (int i = 0; i < piece_count; ++i) {
    const double t = durations[i];
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    Eigen::Matrix<double, 6, 6> system;
    system << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, t,
        t2, t3, t4, t5, 0, 1, 2 * t, 3 * t2, 4 * t3, 5 * t4, 0, 0, 2, 6 * t,
        12 * t2, 20 * t3;

    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 6, 6>> qr =
        system.colPivHouseholderQr();

    PolyPiece piece;
    piece.duration = t;
    for (int axis = 0; axis < 2; ++axis) {
      Eigen::Matrix<double, 6, 1> rhs;
      rhs(0) = positions[i](axis);
      rhs(1) = velocities[i](axis);
      rhs(2) = accelerations[i](axis);
      rhs(3) = positions[i + 1](axis);
      rhs(4) = velocities[i + 1](axis);
      rhs(5) = accelerations[i + 1](axis);

      const Eigen::Matrix<double, 6, 1> coeffs = qr.solve(rhs);
      for (int j = 0; j < 6; ++j) {
        const double value = coeffs(j);
        piece.coeffs(axis, j) = std::isfinite(value) ? value : 0.0;
      }
    }
    pieces[i] = piece;
  }
}

void TrajectoryOptimizer::fitYawQuintic(
    const std::vector<double>& yaws,
    const std::vector<double>& durations,
    double omega0, double omegaf,
    std::vector<YawPolyPiece>& pieces) {
  const int piece_count = static_cast<int>(yaws.size()) - 1;
  if (piece_count <= 0) {
    pieces.clear();
    return;
  }

  std::vector<double> omegas(piece_count + 1, 0.0);
  std::vector<double> alphas(piece_count + 1, 0.0);
  omegas[0] = omega0;
  omegas[piece_count] = omegaf;

  for (int i = 1; i < piece_count; ++i) {
    double dt_sum = durations[i - 1] + durations[i];
    if (dt_sum < 1e-12) {
      dt_sum = 1e-12;
    }
    const double yaw_delta = normalizeAngle(yaws[i + 1] - yaws[i - 1]);
    omegas[i] = yaw_delta / dt_sum;
    alphas[i] = 0.0;
  }

  pieces.resize(piece_count);
  for (int i = 0; i < piece_count; ++i) {
    const double t = durations[i];
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    Eigen::Matrix<double, 6, 6> system;
    system << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, t,
        t2, t3, t4, t5, 0, 1, 2 * t, 3 * t2, 4 * t3, 5 * t4, 0, 0, 2, 6 * t,
        12 * t2, 20 * t3;

    Eigen::Matrix<double, 6, 1> rhs;
    rhs(0) = yaws[i];
    rhs(1) = omegas[i];
    rhs(2) = alphas[i];
    rhs(3) = yaws[i] + normalizeAngle(yaws[i + 1] - yaws[i]);
    rhs(4) = omegas[i + 1];
    rhs(5) = alphas[i + 1];

    const Eigen::Matrix<double, 6, 1> coeffs =
        system.colPivHouseholderQr().solve(rhs);

    YawPolyPiece piece;
    piece.duration = t;
    for (int j = 0; j < 6; ++j) {
      piece.coeffs(0, j) = coeffs(j);
    }
    pieces[i] = piece;
  }
}

Trajectory TrajectoryOptimizer::smoothStitchedTrajectory(
    const Trajectory& trajectory) {
  if (trajectory.empty() || svsdf_ == nullptr ||
      params_.post_stitch_smoothing_iterations <= 0 ||
      params_.post_stitch_smoothing_alpha <= 0.0) {
    return trajectory;
  }

  const double total_duration = trajectory.totalDuration();
  if (total_duration <= 0.0) return trajectory;
  const double spacing = params_.post_stitch_sample_spacing;
  const int dense_samples =
      std::max(2, static_cast<int>(std::ceil(total_duration / 0.02)));
  std::vector<SE2State> states;
  states.reserve(static_cast<size_t>(dense_samples) + 1);
  SE2State last_dense = trajectory.sample(0.0);
  states.push_back(last_dense);
  double accumulated_distance = 0.0;
  for (int sample = 1; sample <= dense_samples; ++sample) {
    const double time = total_duration * static_cast<double>(sample) /
                        static_cast<double>(dense_samples);
    const SE2State current = trajectory.sample(time);
    accumulated_distance +=
        (current.position() - last_dense.position()).norm();
    if (accumulated_distance >= spacing || sample == dense_samples) {
      states.push_back(current);
      accumulated_distance = 0.0;
    }
    last_dense = current;
  }
  if (states.size() < 3) return trajectory;

  const auto connection_free = [&](const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& goal) {
    const Eigen::Vector2d delta = goal - start;
    const double length = delta.norm();
    if (length < 1e-9) return true;
    const double yaw = std::atan2(delta.y(), delta.x());
    const int samples =
        std::max(1, static_cast<int>(std::ceil(length / 0.025)));
    for (int sample = 0; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) /
                           static_cast<double>(samples);
      const Eigen::Vector2d position = start + ratio * delta;
      if (svsdf_->evaluate(SE2State(position.x(), position.y(), yaw)) < 0.0) {
        return false;
      }
    }
    return true;
  };

  std::vector<SE2State> smoothed = states;
  for (int iteration = 0;
       iteration < params_.post_stitch_smoothing_iterations; ++iteration) {
    std::vector<SE2State> next = smoothed;
    for (size_t index = 1; index + 1 < smoothed.size(); ++index) {
      const Eigen::Vector2d previous = smoothed[index - 1].position();
      const Eigen::Vector2d current = smoothed[index].position();
      const Eigen::Vector2d following = smoothed[index + 1].position();
      const Eigen::Vector2d target =
          (1.0 - params_.post_stitch_smoothing_alpha) * current +
          0.5 * params_.post_stitch_smoothing_alpha *
              (previous + following);
      const Eigen::Vector2d tangent = following - previous;
      if (tangent.norm() < 1e-8) continue;
      const double yaw = std::atan2(tangent.y(), tangent.x());
      if (svsdf_->evaluate(SE2State(target.x(), target.y(), yaw)) < 0.0 ||
          !connection_free(previous, target) ||
          !connection_free(target, following)) {
        continue;
      }
      next[index] = SE2State(target.x(), target.y(), yaw);
    }
    smoothed.swap(next);
  }

  smoothed.front() = states.front();
  smoothed.back() = states.back();
  for (size_t index = 1; index + 1 < smoothed.size(); ++index) {
    const Eigen::Vector2d tangent =
        smoothed[index + 1].position() - smoothed[index - 1].position();
    if (tangent.norm() > 1e-8) {
      smoothed[index].yaw = std::atan2(tangent.y(), tangent.x());
    }
  }

  std::vector<Eigen::Vector2d> positions(smoothed.size());
  std::vector<double> yaws(smoothed.size());
  for (size_t index = 0; index < smoothed.size(); ++index) {
    positions[index] = smoothed[index].position();
    yaws[index] = smoothed[index].yaw;
  }
  const std::vector<double> durations =
      allocateTime(smoothed, total_duration);
  Trajectory candidate;
  fitMincoQuintic(positions, durations, Eigen::Vector2d::Zero(),
                  Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
                  Eigen::Vector2d::Zero(), candidate.pos_pieces);
  fitYawQuintic(yaws, durations, 0.0, 0.0, candidate.yaw_pieces);
  if (candidate.empty()) return trajectory;

  const int validation_samples = std::max(
      2, static_cast<int>(std::ceil(candidate.totalDuration() / 0.01)));
  for (int sample = 0; sample <= validation_samples; ++sample) {
    const double time = candidate.totalDuration() *
                        static_cast<double>(sample) /
                        static_cast<double>(validation_samples);
    if (svsdf_->evaluate(candidate.sample(time)) < 0.0) {
      return trajectory;
    }
  }

  double maximum_velocity = 0.0;
  double maximum_acceleration = 0.0;
  double maximum_yaw_rate = 0.0;
  for (int sample = 0; sample <= validation_samples; ++sample) {
    const double time = candidate.totalDuration() *
                        static_cast<double>(sample) /
                        static_cast<double>(validation_samples);
    maximum_velocity =
        std::max(maximum_velocity, candidate.sampleVelocity(time).norm());
    maximum_acceleration = std::max(
        maximum_acceleration, candidate.sampleAcceleration(time).norm());
    maximum_yaw_rate =
        std::max(maximum_yaw_rate, std::abs(candidate.sampleYawRate(time)));
  }
  double time_scale = 1.0;
  time_scale = std::max(
      time_scale, maximum_velocity / std::max(1e-6, params_.max_vel));
  time_scale = std::max(
      time_scale,
      std::sqrt(maximum_acceleration / std::max(1e-6, params_.max_acc)));
  time_scale = std::max(
      time_scale, maximum_yaw_rate / std::max(1e-6, params_.max_yaw_rate));
  if (time_scale > 1.0 + 1e-6) {
    time_scale *= 1.05;
    for (PolyPiece& piece : candidate.pos_pieces) {
      piece.duration *= time_scale;
      double divisor = time_scale;
      for (int order = 1; order < piece.coeffs.cols(); ++order) {
        piece.coeffs.col(order) /= divisor;
        divisor *= time_scale;
      }
    }
    for (YawPolyPiece& piece : candidate.yaw_pieces) {
      piece.duration *= time_scale;
      double divisor = time_scale;
      for (int order = 1; order < piece.coeffs.cols(); ++order) {
        piece.coeffs.col(order) /= divisor;
        divisor *= time_scale;
      }
    }
  }
  return candidate;
}

Trajectory TrajectoryOptimizer::optimizeSE2(
    const std::vector<SE2State>& waypoints, double total_time) {
  const int waypoint_count = static_cast<int>(waypoints.size());
  if (waypoint_count < 2) {
    return Trajectory();
  }

  std::vector<SE2State> optimized_waypoints = waypoints;
  std::vector<double> durations = allocateTime(optimized_waypoints, total_time);
  const double step = params_.step_size;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    std::vector<Eigen::Vector2d> positions(waypoint_count);
    std::vector<double> yaws(waypoint_count, 0.0);
    for (int i = 0; i < waypoint_count; ++i) {
      positions[i] = Eigen::Vector2d(optimized_waypoints[i].x, optimized_waypoints[i].y);
      yaws[i] = optimized_waypoints[i].yaw;
    }

    double total_grad_norm = 0.0;
    for (int i = 1; i < waypoint_count - 1; ++i) {
      Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
      double grad_yaw = 0.0;

      const double dt_prev = durations[i - 1];
      const double dt_next = durations[i];
      const double dt_avg = std::max(0.05, 0.5 * (dt_prev + dt_next));

      const Eigen::Vector2d acc_approx =
          (positions[i + 1] - 2.0 * positions[i] + positions[i - 1]) /
          (dt_avg * dt_avg);
      Eigen::Vector2d grad_smooth = -4.0 * acc_approx / (dt_avg * dt_avg);
      const double smooth_norm = grad_smooth.norm();
      if (smooth_norm > 100.0) {
        grad_smooth *= 100.0 / smooth_norm;
      }
      grad_pos += params_.lambda_smooth * grad_smooth;

      const double yaw_dd =
          (normalizeAngle(yaws[i + 1] - yaws[i]) -
           normalizeAngle(yaws[i] - yaws[i - 1])) /
          (dt_avg * dt_avg);
      double yaw_smooth_grad =
          params_.lambda_smooth * (-4.0 * yaw_dd / (dt_avg * dt_avg));
      if (std::abs(yaw_smooth_grad) > 100.0) {
        yaw_smooth_grad = (yaw_smooth_grad > 0.0) ? 100.0 : -100.0;
      }
      grad_yaw += yaw_smooth_grad;

      if (svsdf_) {
        const double sdf_value = svsdf_->evaluate(optimized_waypoints[i]);
        if (sdf_value < params_.safety_margin) {
          Eigen::Vector2d grad_pos_sdf;
          double grad_yaw_sdf = 0.0;
          svsdf_->gradient(optimized_waypoints[i], grad_pos_sdf, grad_yaw_sdf);
          const double penalty = params_.safety_margin - sdf_value;
          grad_pos -= params_.lambda_safety * 2.0 * penalty * grad_pos_sdf;
          grad_yaw -= params_.lambda_safety * 2.0 * penalty * grad_yaw_sdf;
        }
      }

      double dt_span = dt_prev + dt_next;
      if (dt_span < 1e-12) {
        dt_span = 1e-12;
      }
      const Eigen::Vector2d vel_est = (positions[i + 1] - positions[i - 1]) / dt_span;
      const double vel_norm = vel_est.norm();
      if (vel_norm > params_.max_vel) {
        const double excess = vel_norm - params_.max_vel;
        const Eigen::Vector2d midpoint = 0.5 * (positions[i - 1] + positions[i + 1]);
        grad_pos += params_.lambda_dynamics * excess *
                    (positions[i] - midpoint) / (vel_norm + 1e-12);
      }

      const double acc_norm = acc_approx.norm();
      if (acc_norm > params_.max_acc) {
        const double excess = acc_norm - params_.max_acc;
        grad_pos += params_.lambda_dynamics * excess *
                    (-2.0 * acc_approx) /
                    (acc_norm * dt_avg * dt_avg + 1e-12);
      }

      const double grad_norm = grad_pos.norm();
      if (grad_norm > 50.0) {
        grad_pos *= 50.0 / grad_norm;
      }
      if (std::abs(grad_yaw) > 50.0) {
        grad_yaw = (grad_yaw > 0.0) ? 50.0 : -50.0;
      }

      optimized_waypoints[i].x -= step * grad_pos.x();
      optimized_waypoints[i].y -= step * grad_pos.y();
      optimized_waypoints[i].yaw =
          normalizeAngle(optimized_waypoints[i].yaw - step * grad_yaw);

      total_grad_norm += grad_pos.squaredNorm() + grad_yaw * grad_yaw;
    }

    durations = allocateTime(optimized_waypoints, total_time);
    if (total_grad_norm < 1e-10) {
      break;
    }
  }

  const double max_piece_len = params_.fit_max_piece_length_se2;
  std::vector<Eigen::Vector2d> fit_positions(waypoint_count);
  for (int i = 0; i < waypoint_count; ++i) {
    fit_positions[i] = optimized_waypoints[i].position();
  }
  std::vector<Eigen::Vector2d> final_pos;
  std::vector<double> final_yaws;
  final_pos.push_back(fit_positions.front());
  final_yaws.push_back(optimized_waypoints[0].yaw);

  double accum_dist = 0.0;
  for (int i = 1; i < waypoint_count; ++i) {
    const double dx = optimized_waypoints[i].x - optimized_waypoints[i - 1].x;
    const double dy = optimized_waypoints[i].y - optimized_waypoints[i - 1].y;
    accum_dist += std::sqrt(dx * dx + dy * dy);
    const bool turn_support = IsTurnSupportPoint(
        fit_positions, i, params_.fit_max_heading_change);
    if (accum_dist >= max_piece_len || turn_support ||
        i == waypoint_count - 1) {
      final_pos.push_back(Eigen::Vector2d(optimized_waypoints[i].x,
                                          optimized_waypoints[i].y));
      final_yaws.push_back(optimized_waypoints[i].yaw);
      accum_dist = 0.0;
    }
  }

  std::vector<SE2State> downsampled(final_pos.size());
  for (size_t i = 0; i < final_pos.size(); ++i) {
    downsampled[i] = SE2State(final_pos[i].x(), final_pos[i].y(), final_yaws[i]);
  }
  const std::vector<double> downsampled_durations = allocateTime(downsampled, total_time);

  Trajectory traj;
  // Segments are optimized independently.  Zero endpoint derivatives provide
  // a common C2 contract across R2/SE2/SVSDF levels; otherwise stitching has
  // to bend the first piece and can create a dynamics spike at a risk boundary.
  fitMincoQuintic(final_pos, downsampled_durations, Eigen::Vector2d::Zero(),
                  Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
                  Eigen::Vector2d::Zero(), traj.pos_pieces);
  fitYawQuintic(final_yaws, downsampled_durations, 0.0, 0.0, traj.yaw_pieces);
  return traj;
}

Trajectory TrajectoryOptimizer::optimizeR2(
    const std::vector<SE2State>& waypoints, double total_time) {
  const int waypoint_count = static_cast<int>(waypoints.size());
  if (waypoint_count < 2) {
    return Trajectory();
  }

  std::vector<SE2State> optimized_waypoints = waypoints;
  const std::vector<SE2State> reference = waypoints;
  std::vector<double> durations = allocateTime(optimized_waypoints, total_time);
  const double step = params_.step_size;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    std::vector<Eigen::Vector2d> positions(waypoint_count);
    for (int i = 0; i < waypoint_count; ++i) {
      positions[i] = Eigen::Vector2d(optimized_waypoints[i].x, optimized_waypoints[i].y);
    }

    double total_grad_norm = 0.0;
    for (int i = 1; i < waypoint_count - 1; ++i) {
      Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();

      const double dt_prev = durations[i - 1];
      const double dt_next = durations[i];
      const double dt_avg = std::max(0.05, 0.5 * (dt_prev + dt_next));

      const Eigen::Vector2d acc_approx =
          (positions[i + 1] - 2.0 * positions[i] + positions[i - 1]) /
          (dt_avg * dt_avg);
      Eigen::Vector2d grad_smooth = -4.0 * acc_approx / (dt_avg * dt_avg);
      const double smooth_norm = grad_smooth.norm();
      if (smooth_norm > 100.0) {
        grad_smooth *= 100.0 / smooth_norm;
      }
      grad_pos += params_.lambda_smooth * grad_smooth;

      if (svsdf_) {
        const SE2State state(optimized_waypoints[i].x, optimized_waypoints[i].y,
                             optimized_waypoints[i].yaw);
        const double sdf_value = svsdf_->evaluate(state);
        if (sdf_value < params_.safety_margin) {
          Eigen::Vector2d grad_pos_sdf;
          double grad_yaw_sdf = 0.0;
          svsdf_->gradient(state, grad_pos_sdf, grad_yaw_sdf);
          const double penalty = params_.safety_margin - sdf_value;
          grad_pos -= params_.lambda_safety * 2.0 * penalty * grad_pos_sdf;
        }
      }

      const Eigen::Vector2d ref_pos(reference[i].x, reference[i].y);
      grad_pos += params_.lambda_pos_residual * 2.0 * (positions[i] - ref_pos);

      const Eigen::Vector2d vel_dir = positions[i + 1] - positions[i - 1];
      const double vel_norm = vel_dir.norm();
      if (vel_norm > 1e-9) {
        const double yaw_curr = std::atan2(vel_dir.y(), vel_dir.x());

        double yaw_prev = optimized_waypoints[0].yaw;
        if (i >= 2) {
          const Eigen::Vector2d prev_vel = positions[i] - positions[i - 2];
          if (prev_vel.norm() > 1e-9) {
            yaw_prev = std::atan2(prev_vel.y(), prev_vel.x());
          }
        }

        double yaw_next = optimized_waypoints[waypoint_count - 1].yaw;
        if (i + 2 < waypoint_count) {
          const Eigen::Vector2d next_vel = positions[i + 2] - positions[i];
          if (next_vel.norm() > 1e-9) {
            yaw_next = std::atan2(next_vel.y(), next_vel.x());
          }
        }

        const double yaw_dd = normalizeAngle(yaw_next - yaw_curr) -
                              normalizeAngle(yaw_curr - yaw_prev);
        Eigen::Vector2d perp(-vel_dir.y(), vel_dir.x());
        perp /= vel_norm;
        grad_pos += params_.lambda_yaw_residual * yaw_dd * perp;
      }

      optimized_waypoints[i].x -= step * grad_pos.x();
      optimized_waypoints[i].y -= step * grad_pos.y();
      total_grad_norm += grad_pos.squaredNorm();
    }

    durations = allocateTime(optimized_waypoints, total_time);
    if (total_grad_norm < 1e-10) {
      break;
    }
  }

  std::vector<Eigen::Vector2d> all_pos(waypoint_count);
  std::vector<double> all_yaws(waypoint_count, 0.0);
  for (int i = 0; i < waypoint_count; ++i) {
    all_pos[i] = Eigen::Vector2d(optimized_waypoints[i].x, optimized_waypoints[i].y);
  }
  all_yaws[0] = optimized_waypoints[0].yaw;
  all_yaws[waypoint_count - 1] = optimized_waypoints[waypoint_count - 1].yaw;
  for (int i = 1; i < waypoint_count - 1; ++i) {
    const Eigen::Vector2d vel_dir = all_pos[i + 1] - all_pos[i - 1];
    if (vel_dir.norm() > 1e-9) {
      all_yaws[i] = std::atan2(vel_dir.y(), vel_dir.x());
    } else {
      all_yaws[i] = all_yaws[i - 1];
    }
  }

  const double max_piece_len = params_.fit_max_piece_length_r2;
  std::vector<Eigen::Vector2d> final_pos;
  std::vector<double> final_yaws;
  final_pos.push_back(all_pos[0]);
  final_yaws.push_back(all_yaws[0]);

  double accum_dist = 0.0;
  for (int i = 1; i < waypoint_count; ++i) {
    accum_dist += (all_pos[i] - all_pos[i - 1]).norm();
    const bool turn_support = IsTurnSupportPoint(
        all_pos, i, params_.fit_max_heading_change);
    if (accum_dist >= max_piece_len || turn_support ||
        i == waypoint_count - 1) {
      final_pos.push_back(all_pos[i]);
      final_yaws.push_back(all_yaws[i]);
      accum_dist = 0.0;
    }
  }

  std::vector<SE2State> downsampled(final_pos.size());
  for (size_t i = 0; i < final_pos.size(); ++i) {
    downsampled[i] = SE2State(final_pos[i].x(), final_pos[i].y(), final_yaws[i]);
  }
  const std::vector<double> downsampled_durations = allocateTime(downsampled, total_time);

  Trajectory traj;
  fitMincoQuintic(final_pos, downsampled_durations, Eigen::Vector2d::Zero(),
                  Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
                  Eigen::Vector2d::Zero(), traj.pos_pieces);
  fitYawQuintic(final_yaws, downsampled_durations, 0.0, 0.0, traj.yaw_pieces);
  return traj;
}

Trajectory TrajectoryOptimizer::stitch(
    const std::vector<MotionSegment>& segments,
    const std::vector<Trajectory>& optimized) {
  Trajectory traj;
  const size_t count = std::min(segments.size(), optimized.size());
  for (size_t i = 0; i < count; ++i) {
    Trajectory seg_traj = optimized[i];
    if (seg_traj.empty()) {
      continue;
    }
    if (!traj.pos_pieces.empty() && !traj.yaw_pieces.empty() &&
        !seg_traj.pos_pieces.empty() && !seg_traj.yaw_pieces.empty()) {
      PolyPiece& previous = traj.pos_pieces.back();
      PolyPiece& current = seg_traj.pos_pieces.front();
      const Eigen::Vector2d zero_acceleration = Eigen::Vector2d::Zero();

      // Ackermann deployment may reserve one guard piece on each side of a
      // solver transition. Conditioning these adjacent internal joins first
      // spreads the heading change across a wider arc instead of concentrating
      // it in the first centimetres of the HIGH segment.
      if (params_.stitch_guard_pieces > 0 && traj.pos_pieces.size() >= 2) {
        PolyPiece& left_guard = traj.pos_pieces[traj.pos_pieces.size() - 2];
        const Eigen::Vector2d guard_velocity =
            SharedGeometricBoundaryVelocity(left_guard, previous, params_);
        if (guard_velocity.squaredNorm() > 1e-12) {
          const Eigen::Vector2d guard_boundary =
              left_guard.evaluate(left_guard.duration);
          left_guard = RefitPositionPiece(
              left_guard, left_guard.evaluate(0.0), left_guard.velocity(0.0),
              left_guard.acceleration(0.0), guard_boundary, guard_velocity,
              zero_acceleration);
          previous = RefitPositionPiece(
              previous, guard_boundary, guard_velocity, zero_acceleration,
              previous.evaluate(previous.duration),
              previous.velocity(previous.duration),
              previous.acceleration(previous.duration));
        }
      }
      if (params_.stitch_guard_pieces > 0 &&
          seg_traj.pos_pieces.size() >= 2) {
        PolyPiece& right_guard = seg_traj.pos_pieces[1];
        const Eigen::Vector2d guard_velocity =
            SharedGeometricBoundaryVelocity(current, right_guard, params_);
        if (guard_velocity.squaredNorm() > 1e-12) {
          const Eigen::Vector2d guard_boundary =
              current.evaluate(current.duration);
          current = RefitPositionPiece(
              current, current.evaluate(0.0), current.velocity(0.0),
              current.acceleration(0.0), guard_boundary, guard_velocity,
              zero_acceleration);
          right_guard = RefitPositionPiece(
              right_guard, guard_boundary, guard_velocity, zero_acceleration,
              right_guard.evaluate(right_guard.duration),
              right_guard.velocity(right_guard.duration),
              right_guard.acceleration(right_guard.duration));
        }
      }

      const Eigen::Vector2d shared_velocity =
          SharedGeometricBoundaryVelocity(previous, current, params_);
      if (shared_velocity.squaredNorm() > 1e-12) {
        const double previous_t = previous.duration;
        const double current_t = current.duration;
        const Eigen::Vector2d boundary = previous.evaluate(previous_t);
        // Refit both sides of the risk boundary.  Their remote endpoint
        // conditions remain unchanged, while the join receives one shared,
        // nonzero tangent and zero shared acceleration.  This is C2 in time
        // and G2 at the geometric join, so a stopped-parametrization cusp can
        // no longer pass as a smooth transition.
        previous = RefitPositionPiece(
            previous, previous.evaluate(0.0), previous.velocity(0.0),
            previous.acceleration(0.0), boundary, shared_velocity,
            zero_acceleration);
        current = RefitPositionPiece(
            current, boundary, shared_velocity, zero_acceleration,
            current.evaluate(current_t), current.velocity(current_t),
            current.acceleration(current_t));
      } else {
        // Degenerate chords and genuine reversals retain the conservative stop
        // boundary; those cases are handled by the fallback/search machinery.
        current = ConditionPositionBoundary(previous, current);
      }
      seg_traj.yaw_pieces.front() =
          ConditionYawBoundary(traj.yaw_pieces.back(),
                               seg_traj.yaw_pieces.front());
    }
    traj.pos_pieces.insert(traj.pos_pieces.end(), seg_traj.pos_pieces.begin(),
                           seg_traj.pos_pieces.end());
    traj.yaw_pieces.insert(traj.yaw_pieces.end(), seg_traj.yaw_pieces.begin(),
                           seg_traj.yaw_pieces.end());
  }
  return smoothStitchedTrajectory(traj);
}



}  // namespace isweep_planner
