#include "common/util/csv_logger.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <system_error>

namespace common {
namespace util {

std::map<std::string, std::ofstream> CSVLogger::files_;
std::string CSVLogger::output_directory_ = "/tmp/buffered_footprint_planner";

void CSVLogger::EnsureDirectoryExists(const std::string& directory) {
  std::error_code error;
  std::filesystem::create_directories(directory, error);
}

void CSVLogger::SetOutputDirectory(const std::string& directory) {
  output_directory_ = directory;
  EnsureDirectoryExists(output_directory_);
}

bool CSVLogger::Open(const std::string& filename, const std::vector<std::string>& header) {
  EnsureDirectoryExists(output_directory_);
  auto& stream = files_[filename];
  stream.open(output_directory_ + "/" + filename);
  if (!stream.is_open()) return false;
  for (std::size_t i = 0; i < header.size(); ++i) {
    if (i != 0) stream << ',';
    stream << header[i];
  }
  stream << '\n';
  return true;
}

void CSVLogger::WriteLine(const std::string& filename, const std::vector<double>& values) {
  const auto found = files_.find(filename);
  if (found == files_.end() || !found->second.is_open()) return;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) found->second << ',';
    found->second << std::setprecision(16) << values[i];
  }
  found->second << '\n';
}

void CSVLogger::CloseAll() {
  for (auto& entry : files_) entry.second.close();
  files_.clear();
}

std::string CSVLogger::CreateTimestampedFilename(const std::string& prefix,
                                                  const std::string& suffix) {
  const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm time_info{};
#ifdef _WIN32
  localtime_s(&time_info, &now);
#else
  localtime_r(&now, &time_info);
#endif
  std::ostringstream stream;
  stream << prefix << '_' << std::put_time(&time_info, "%Y%m%d_%H%M%S") << suffix;
  return stream.str();
}

bool HybridAStarLogger::initialized_ = false;
std::string HybridAStarLogger::run_id_;

void HybridAStarLogger::Initialize(const std::string& run_id) {
  Finalize();
  run_id_ = run_id.empty() ? CSVLogger::CreateTimestampedFilename("hybrid_astar", "") : run_id;
  CSVLogger::SetOutputDirectory("/tmp/buffered_footprint_planner/hybrid_astar");
  CSVLogger::Open(run_id_ + "_search_nodes.csv",
                  {"x", "y", "phi", "trajectory_cost", "heuristic_cost", "total_cost",
                   "direction", "steering", "iteration"});
  CSVLogger::Open(run_id_ + "_rs_path_summary.csv", {"path_id", "total_length", "num_points"});
  CSVLogger::Open(run_id_ + "_coarse_path.csv", {"index", "x", "y", "phi", "v", "a", "steer"});
  initialized_ = true;
}

void HybridAStarLogger::LogSearchNode(double x, double y, double phi, double trajectory_cost,
                                      double heuristic_cost, bool direction, double steering,
                                      int iteration) {
  if (!initialized_) return;
  CSVLogger::WriteLine(run_id_ + "_search_nodes.csv",
                       {x, y, phi, trajectory_cost, heuristic_cost,
                        trajectory_cost + heuristic_cost, direction ? 1.0 : 0.0,
                        steering, static_cast<double>(iteration)});
}

void HybridAStarLogger::LogReedSheppPath(const std::vector<double>& x,
                                         const std::vector<double>&,
                                         const std::vector<double>&,
                                         const std::vector<bool>&,
                                         double total_length) {
  if (!initialized_) return;
  static std::size_t path_id = 0;
  ++path_id;
  CSVLogger::WriteLine(run_id_ + "_rs_path_summary.csv",
                       {static_cast<double>(path_id), total_length, static_cast<double>(x.size())});
}

void HybridAStarLogger::LogCoarsePath(const std::vector<double>& x,
                                      const std::vector<double>& y,
                                      const std::vector<double>& phi,
                                      const std::vector<double>& v,
                                      const std::vector<double>& a,
                                      const std::vector<double>& steer) {
  if (!initialized_) return;
  for (std::size_t i = 0; i < x.size(); ++i) {
    CSVLogger::WriteLine(run_id_ + "_coarse_path.csv",
                         {static_cast<double>(i), x[i], y[i], phi[i],
                          i < v.size() ? v[i] : 0.0,
                          i < a.size() ? a[i] : 0.0,
                          i < steer.size() ? steer[i] : 0.0});
  }
}

void HybridAStarLogger::Finalize() {
  if (!initialized_) return;
  CSVLogger::CloseAll();
  initialized_ = false;
}

}  // namespace util
}  // namespace common
