#include "planner_experiments/geometry_consistency_checker.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <utility>

#include <igl/read_triangle_mesh.h>

namespace planner_experiments {
namespace {

double PolygonArea(const std::vector<Eigen::Vector2d>& polygon) {
  double twice_area = 0.0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const Eigen::Vector2d& a = polygon[i];
    const Eigen::Vector2d& b = polygon[(i + 1) % polygon.size()];
    twice_area += a.x() * b.y() - b.x() * a.y();
  }
  return 0.5 * std::abs(twice_area);
}

Eigen::Vector2d PolygonCentroid(const std::vector<Eigen::Vector2d>& polygon) {
  double cross_sum = 0.0;
  Eigen::Vector2d weighted = Eigen::Vector2d::Zero();
  for (size_t i = 0; i < polygon.size(); ++i) {
    const Eigen::Vector2d& a = polygon[i];
    const Eigen::Vector2d& b = polygon[(i + 1) % polygon.size()];
    const double cross = a.x() * b.y() - b.x() * a.y();
    cross_sum += cross;
    weighted += (a + b) * cross;
  }
  if (std::abs(cross_sum) < 1e-12) return Eigen::Vector2d::Zero();
  return weighted / (3.0 * cross_sum);
}

double NearestDistance(const Eigen::Vector2d& point,
                       const std::vector<Eigen::Vector2d>& samples) {
  double best = std::numeric_limits<double>::infinity();
  for (const Eigen::Vector2d& candidate : samples) {
    best = std::min(best, (point - candidate).norm());
  }
  return best;
}

}  // namespace

std::vector<Eigen::Vector2d> GeometryConsistencyChecker::SampleClosedOutline(
    const std::vector<Eigen::Vector2d>& polygon, double step) {
  std::vector<Eigen::Vector2d> samples;
  if (polygon.size() < 2) return samples;
  const double spacing = std::max(1e-5, step);
  for (size_t i = 0; i < polygon.size(); ++i) {
    const Eigen::Vector2d& a = polygon[i];
    const Eigen::Vector2d& b = polygon[(i + 1) % polygon.size()];
    const double length = (b - a).norm();
    const int count = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int j = 0; j < count; ++j) {
      samples.push_back(a + (b - a) * (static_cast<double>(j) / count));
    }
  }
  return samples;
}

GeometryConsistencyResult GeometryConsistencyChecker::Check(
    const std::vector<Eigen::Vector2d>& polygon,
    const std::string& mesh_path,
    const GeometryCheckConfig& config) const {
  GeometryConsistencyResult result;
  if (!config.enabled) {
    result.consistency_passed = true;
    return result;
  }
  if (polygon.size() < 3) {
    result.failure_reason = "polygon has fewer than three vertices";
    return result;
  }

  Eigen::MatrixXd vertices;
  Eigen::MatrixXi faces;
  if (!igl::read_triangle_mesh(mesh_path, vertices, faces) ||
      vertices.cols() < 2 || faces.cols() < 3) {
    result.failure_reason = "mesh loader failed: " + mesh_path;
    return result;
  }

  result.polygon_area = PolygonArea(polygon);
  const Eigen::Vector2d polygon_centroid = PolygonCentroid(polygon);

  std::vector<int> positive_faces;
  std::vector<int> negative_faces;
  for (Eigen::Index i = 0; i < faces.rows(); ++i) {
    const Eigen::Vector2d a = vertices.row(faces(i, 0)).head<2>();
    const Eigen::Vector2d b = vertices.row(faces(i, 1)).head<2>();
    const Eigen::Vector2d c = vertices.row(faces(i, 2)).head<2>();
    const double cross = (b.x() - a.x()) * (c.y() - a.y()) -
                         (b.y() - a.y()) * (c.x() - a.x());
    if (cross > 1e-12) positive_faces.push_back(static_cast<int>(i));
    if (cross < -1e-12) negative_faces.push_back(static_cast<int>(i));
  }
  const std::vector<int>& projected_faces =
      !positive_faces.empty() ? positive_faces : negative_faces;
  if (projected_faces.empty()) {
    result.failure_reason = "mesh has no non-degenerate XY projection";
    return result;
  }

  std::map<std::pair<int, int>, int> edge_counts;
  Eigen::Vector2d mesh_weighted_centroid = Eigen::Vector2d::Zero();
  double mesh_area = 0.0;
  for (int face_index : projected_faces) {
    const int ids[3] = {faces(face_index, 0), faces(face_index, 1),
                        faces(face_index, 2)};
    const Eigen::Vector2d a = vertices.row(ids[0]).head<2>();
    const Eigen::Vector2d b = vertices.row(ids[1]).head<2>();
    const Eigen::Vector2d c = vertices.row(ids[2]).head<2>();
    const double area = 0.5 * std::abs(
        (b.x() - a.x()) * (c.y() - a.y()) -
        (b.y() - a.y()) * (c.x() - a.x()));
    mesh_area += area;
    mesh_weighted_centroid += area * (a + b + c) / 3.0;
    for (int e = 0; e < 3; ++e) {
      const int first = std::min(ids[e], ids[(e + 1) % 3]);
      const int second = std::max(ids[e], ids[(e + 1) % 3]);
      ++edge_counts[std::make_pair(first, second)];
    }
  }
  result.mesh_projected_area = mesh_area;
  if (mesh_area <= 1e-12) {
    result.failure_reason = "mesh projected area is zero";
    return result;
  }
  const Eigen::Vector2d mesh_centroid = mesh_weighted_centroid / mesh_area;
  result.centroid_error_m = (polygon_centroid - mesh_centroid).norm();

  std::vector<Eigen::Vector2d> mesh_outline_samples;
  const double step = std::max(1e-5, config.outline_sample_step_m);
  for (const auto& edge : edge_counts) {
    if (edge.second != 1) continue;
    const Eigen::Vector2d a = vertices.row(edge.first.first).head<2>();
    const Eigen::Vector2d b = vertices.row(edge.first.second).head<2>();
    const int count = std::max(1, static_cast<int>(std::ceil((b - a).norm() / step)));
    for (int i = 0; i < count; ++i) {
      mesh_outline_samples.push_back(
          a + (b - a) * (static_cast<double>(i) / count));
    }
  }
  const std::vector<Eigen::Vector2d> polygon_outline_samples =
      SampleClosedOutline(polygon, step);
  if (mesh_outline_samples.empty() || polygon_outline_samples.empty()) {
    result.failure_reason = "could not extract projected mesh outline";
    return result;
  }

  double polygon_to_mesh_sum = 0.0;
  double mesh_to_polygon_sum = 0.0;
  double hausdorff = 0.0;
  double radial_error = 0.0;
  for (const Eigen::Vector2d& point : polygon_outline_samples) {
    const double distance = NearestDistance(point, mesh_outline_samples);
    polygon_to_mesh_sum += distance;
    hausdorff = std::max(hausdorff, distance);
    double best_radial = std::numeric_limits<double>::infinity();
    for (const Eigen::Vector2d& mesh_point : mesh_outline_samples) {
      best_radial = std::min(
          best_radial,
          std::abs((point - polygon_centroid).norm() -
                   (mesh_point - mesh_centroid).norm()));
    }
    radial_error = std::max(radial_error, best_radial);
  }
  for (const Eigen::Vector2d& point : mesh_outline_samples) {
    const double distance = NearestDistance(point, polygon_outline_samples);
    mesh_to_polygon_sum += distance;
    hausdorff = std::max(hausdorff, distance);
  }
  result.bidirectional_outline_distance_m =
      0.5 * (polygon_to_mesh_sum / polygon_outline_samples.size() +
             mesh_to_polygon_sum / mesh_outline_samples.size());
  result.hausdorff_distance_m = hausdorff;
  result.maximum_radial_error_m = radial_error;

  const double relative_area_error =
      std::abs(mesh_area - result.polygon_area) /
      std::max(1e-12, result.polygon_area);
  result.consistency_passed =
      result.centroid_error_m <= config.max_centroid_error_m &&
      result.hausdorff_distance_m <= config.max_hausdorff_distance_m &&
      relative_area_error <= config.max_relative_area_error;
  if (!result.consistency_passed) {
    std::ostringstream message;
    message << "geometry mismatch: centroid=" << result.centroid_error_m
            << " hausdorff=" << result.hausdorff_distance_m
            << " relative_area_error=" << relative_area_error;
    result.failure_reason = message.str();
  }
  return result;
}

}  // namespace planner_experiments
