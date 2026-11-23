#include "planner_experiments/result_writer.hpp"

#include <openssl/sha.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace planner_experiments {
namespace {

bool EnsureDirectory(const std::string& path, std::string* error) {
  std::string current;
  if (!path.empty() && path.front() == '/') current = "/";
  std::stringstream stream(path);
  std::string component;
  while (std::getline(stream, component, '/')) {
    if (component.empty()) continue;
    if (!current.empty() && current.back() != '/') current.push_back('/');
    current += component;
    if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
      if (error) *error = "mkdir failed for " + current + ": " + std::strerror(errno);
      return false;
    }
  }
  return true;
}

bool AtomicWrite(const std::string& path, const std::string& contents,
                 std::string* error) {
  const std::string temporary = path + ".tmp";
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) {
      if (error) *error = "cannot open " + temporary;
      return false;
    }
    output << contents;
    output.flush();
    if (!output.good()) {
      if (error) *error = "write failed for " + temporary;
      return false;
    }
  }
  if (std::rename(temporary.c_str(), path.c_str()) != 0) {
    if (error) *error = "rename failed for " + path + ": " + std::strerror(errno);
    return false;
  }
  return true;
}

std::string Hex(const unsigned char* bytes, size_t count) {
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (size_t i = 0; i < count; ++i) result << std::setw(2) << int(bytes[i]);
  return result.str();
}

}  // namespace

std::string ResultWriter::Sha256File(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) return std::string();
  SHA256_CTX context;
  SHA256_Init(&context);
  char buffer[32768];
  while (input.good()) {
    input.read(buffer, sizeof(buffer));
    const std::streamsize count = input.gcount();
    if (count > 0) SHA256_Update(&context, buffer, static_cast<size_t>(count));
  }
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256_Final(digest, &context);
  return Hex(digest, SHA256_DIGEST_LENGTH);
}

std::string ResultWriter::StableRunKey(const PlanningRequest& request,
                                       const std::string& configuration_hash) {
  std::ostringstream canonical;
  canonical << request.method_name << '\n' << request.map_hash << '\n'
            << request.robot_shape_name << '\n' << request.task_id << '\n'
            << request.random_seed << '\n' << std::setprecision(17)
            << request.start.x << ',' << request.start.y << ',' << request.start.theta
            << '\n' << request.goal.x << ',' << request.goal.y << ','
            << request.goal.theta << '\n' << configuration_hash;
  const std::string text = canonical.str();
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(text.data()), text.size(), digest);
  return Hex(digest, SHA256_DIGEST_LENGTH);
}

bool ResultWriter::WriteRunDirectory(const std::string& directory,
                                     const PlanningResult& result,
                                     std::string* error) const {
  if (!EnsureDirectory(directory, error)) return false;
  const nlohmann::json record = ToJson(result.metrics);
  if (!AtomicWrite(directory + "/result.json", record.dump(2) + "\n", error)) {
    return false;
  }

  std::ostringstream states;
  for (const StateMetric& state : result.state_metrics) {
    states << ToJson(state).dump() << '\n';
  }
  if (!AtomicWrite(directory + "/state_metrics.jsonl", states.str(), error)) {
    return false;
  }

  std::ostringstream segments;
  for (const SegmentMetric& segment : result.segment_metrics) {
    segments << ToJson(segment).dump() << '\n';
  }
  if (!AtomicWrite(directory + "/segment_metrics.jsonl", segments.str(), error)) {
    return false;
  }

  std::ostringstream incumbent_events;
  for (const IncumbentEventMetric& event : result.incumbent_events) {
    incumbent_events << ToJson(event).dump() << '\n';
  }
  if (!AtomicWrite(directory + "/incumbent_events.jsonl",
                   incumbent_events.str(), error)) {
    return false;
  }

  std::ostringstream risk_ground_truth;
  for (const RiskGroundTruthMetric& metric : result.risk_ground_truth_metrics) {
    risk_ground_truth << ToJson(metric).dump() << '\n';
  }
  if (!AtomicWrite(directory + "/risk_ground_truth.jsonl",
                   risk_ground_truth.str(), error)) {
    return false;
  }

  std::ostringstream trajectory;
  trajectory << "t,x,y,yaw,vx,vy,ax,ay,yaw_rate,yaw_acceleration\n";
  trajectory << std::setprecision(17);
  for (const TrajectorySample& sample : result.metrics.validation.samples) {
    trajectory << sample.t << ',' << sample.x << ',' << sample.y << ','
               << sample.yaw << ',' << sample.vx << ',' << sample.vy << ','
               << sample.ax << ',' << sample.ay << ',' << sample.yaw_rate << ','
               << sample.yaw_acceleration << '\n';
  }
  if (!AtomicWrite(directory + "/trajectory.csv", trajectory.str(), error)) {
    return false;
  }

  return AtomicWrite(directory + "/complete", "complete\n", error);
}

}  // namespace planner_experiments
