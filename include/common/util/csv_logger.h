#pragma once

#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace common {
namespace util {

class CSVLogger {
 public:
  static bool Open(const std::string& filename, const std::vector<std::string>& header);
  static void WriteLine(const std::string& filename, const std::vector<double>& values);
  static void CloseAll();
  static void SetOutputDirectory(const std::string& directory);
  static std::string CreateTimestampedFilename(const std::string& prefix,
                                               const std::string& suffix = ".csv");

 private:
  static void EnsureDirectoryExists(const std::string& directory);
  static std::map<std::string, std::ofstream> files_;
  static std::string output_directory_;
};

class HybridAStarLogger {
 public:
  static void Initialize(const std::string& run_id = "");
  static void LogSearchNode(double x, double y, double phi, double trajectory_cost,
                            double heuristic_cost, bool direction, double steering,
                            int iteration);
  static void LogReedSheppPath(const std::vector<double>& x,
                               const std::vector<double>& y,
                               const std::vector<double>& phi,
                               const std::vector<bool>& gear,
                               double total_length);
  static void LogCoarsePath(const std::vector<double>& x,
                            const std::vector<double>& y,
                            const std::vector<double>& phi,
                            const std::vector<double>& v,
                            const std::vector<double>& a,
                            const std::vector<double>& steer);
  static void Finalize();

 private:
  static bool initialized_;
  static std::string run_id_;
};

}  // namespace util
}  // namespace common
