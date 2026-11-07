#include "optimizer/embodied_footprint.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <vector>

namespace optimizer {
namespace {

double FrontLength(const VehicleParameter& vehicle) {
  return vehicle.wheel_base + vehicle.front_hang;
}

}  // namespace

double FootprintValidity::max_violation() const {
  return std::max({0.0, arc, forward_cond1, forward_cond2,
                   reverse_cond1, reverse_cond2});
}

FootprintInterval ComputeEmbodiedFootprint(const TrajectoryNode& node,
                                           double dt,
                                           const VehicleParameter& vehicle) {
  FootprintInterval result;
  result.dt = dt;
  result.s = node.v * dt;
  result.gear = result.s >= 0.0;
  result.k = std::tan(node.phy) / vehicle.wheel_base;
  result.splus = std::max(result.s, 0.0);
  result.sminus = std::max(-result.s, 0.0);
  const double abs_k = std::abs(result.k);
  const double half_width = vehicle.width / 2.0;
  const double front = FrontLength(vehicle);

  result.up = result.splus + half_width * abs_k * result.splus;
  result.down = result.sminus + half_width * abs_k * result.sminus;
  result.left =
      std::max(-vehicle.rear_hang * result.k * result.splus,
               (front + 0.5 * result.splus) * result.k * result.splus) +
      std::max(-front * result.k * result.sminus,
               (vehicle.rear_hang + 0.5 * result.sminus) * result.k *
                   result.sminus);
  result.right =
      std::max(vehicle.rear_hang * result.k * result.splus,
               -(front + 0.5 * result.splus) * result.k * result.splus) +
      std::max(front * result.k * result.sminus,
               -(vehicle.rear_hang + 0.5 * result.sminus) * result.k *
                   result.sminus);

  const double c = std::cos(node.theta);
  const double s = std::sin(node.theta);
  result.corners[0] = {node.x + (front + result.up) * c -
                                    (half_width + result.left) * s,
                       node.y + (front + result.up) * s +
                                    (half_width + result.left) * c};
  result.corners[1] = {node.x + (front + result.up) * c +
                                    (half_width + result.right) * s,
                       node.y + (front + result.up) * s -
                                    (half_width + result.right) * c};
  result.corners[2] = {node.x - (vehicle.rear_hang + result.down) * c +
                                    (half_width + result.right) * s,
                       node.y - (vehicle.rear_hang + result.down) * s -
                                    (half_width + result.right) * c};
  result.corners[3] = {node.x - (vehicle.rear_hang + result.down) * c -
                                    (half_width + result.left) * s,
                       node.y - (vehicle.rear_hang + result.down) * s +
                                    (half_width + result.left) * c};
  return result;
}

FootprintValidity EvaluateFootprintValidity(
    const FootprintInterval& footprint, const VehicleParameter& vehicle,
    double eta) {
  const double abs_k = std::abs(footprint.k);
  const double half_width = vehicle.width / 2.0;
  const double front = FrontLength(vehicle);
  FootprintValidity result;
  result.arc = abs_k * (footprint.splus + footprint.sminus) -
               eta * M_PI_2;
  result.forward_cond1 =
      (1.0 + half_width * abs_k) *
          std::tan(abs_k * footprint.splus) -
      eta * vehicle.rear_hang * abs_k;
  result.forward_cond2 =
      abs_k * front * std::tan(abs_k * footprint.splus) -
      eta * (1.0 + half_width * abs_k);
  result.reverse_cond1 =
      (1.0 + half_width * abs_k) *
          std::tan(abs_k * footprint.sminus) -
      eta * front * abs_k;
  result.reverse_cond2 =
      abs_k * vehicle.rear_hang *
          std::tan(abs_k * footprint.sminus) -
      eta * (1.0 + half_width * abs_k);
  return result;
}

bool CheckVehicleGeometryCondition(const VehicleParameter& vehicle,
                                   std::string* error) {
  const double condition = vehicle.width * 0.5 *
                           std::tan(vehicle.delta_max) /
                           vehicle.wheel_base;
  if (condition < 1.0) return true;
  if (error) {
    *error = "Vehicle violates hlb*tan(phy_max)/lw < 1";
  }
  return false;
}

bool WriteGeometryValidation(const VehicleParameter& vehicle,
                             const std::string& output_file,
                             std::string* error) {
  std::ofstream stream(output_file);
  if (!stream) {
    if (error) *error = "Cannot create geometry validation file";
    return false;
  }
  stream << std::setprecision(12);
  stream << "case,v_mps,dt_s,phy_rad,metric,matlab_reference,cpp_value,"
            "absolute_error,validity_pass,reference_source\n";
  struct Case { const char* name; double v; double dt; double phy; };
  const std::vector<Case> cases{
      {"forward_straight", 0.20, 0.20, 0.0},
      {"reverse_straight", -0.18, 0.20, 0.0},
      {"forward_left", 0.18, 0.20, 0.20},
      {"forward_right", 0.18, 0.20, -0.20},
      {"reverse_left", -0.16, 0.20, 0.20},
      {"near_arc_bound", 0.20, 0.25, vehicle.delta_max}};
  // Produced by an actual MATLAB batch execution of the NLP1.mod equations
  // with the current small-car geometry (lw=.143, LF=.179, lr=.032,
  // hlb=.0955). Keep this table independent of the C++ implementation so the
  // generated CSV is a real cross-language comparison, not a self-check.
  const std::array<std::array<double, 11>, 6> matlab{{
      {{0.0, 0.04, 0.04, 0.0, 0.0, 0.0, -1.5707963267949, 0.0, -1.0, 0.0, -1.0}},
      {{0.0, -0.036, 0.0, 0.036, 0.0, 0.0, -1.5707963267949, 0.0, -1.0, 0.0, -1.0}},
      {{1.41755269586484, 0.036, 0.0408735461683833, 0.0,
        0.0100532837190735, 0.0016330207056363, -1.51976442974376,
        0.0126290689304575, -1.12241609778155, -0.253741932559807,
        -1.13537628245509}},
      {{-1.41755269586484, 0.036, 0.0408735461683833, 0.0,
        0.0016330207056363, 0.0100532837190735, -1.51976442974376,
        0.0126290689304575, -1.12241609778155, -0.253741932559807,
        -1.13537628245509}},
      {{1.41755269586484, -0.032, 0.0, 0.036332041038563,
        0.0021773609408484, 0.00811974184191382, -1.52543464052722,
        -0.045361686267675, -1.13537628245509, -0.202203995417316,
        -1.1333171873589}},
      {{2.54551684121417, 0.05, 0.0621548429167977, 0.0,
        0.0259642717803846, 0.00407282694594268, -1.44352048473419,
        0.0776195548587212, -1.18478874962152, -0.455647514577337,
        -1.24309685833595}}}};
  const std::array<const char*, 11> metric_names{{
      "k_1pm", "s_m", "up_m", "down_m", "left_m", "right_m",
      "EF_arc_bound", "EF_forward_cond1", "EF_forward_cond2",
      "EF_reverse_cond1", "EF_reverse_cond2"}};
  for (std::size_t case_index = 0; case_index < cases.size(); ++case_index) {
    const auto& item = cases[case_index];
    TrajectoryNode node;
    node.v = item.v;
    node.phy = item.phy;
    const auto footprint = ComputeEmbodiedFootprint(node, item.dt, vehicle);
    const auto validity = EvaluateFootprintValidity(footprint, vehicle);
    const std::array<double, 11> cpp{{
        footprint.k, footprint.s, footprint.up, footprint.down,
        footprint.left, footprint.right, validity.arc,
        validity.forward_cond1, validity.forward_cond2,
        validity.reverse_cond1, validity.reverse_cond2}};
    const bool validity_pass = validity.max_violation() <= 0.0;
    for (std::size_t metric = 0; metric < cpp.size(); ++metric) {
      stream << item.name << ',' << item.v << ',' << item.dt << ','
             << item.phy << ',' << metric_names[metric] << ','
             << matlab[case_index][metric] << ',' << cpp[metric] << ','
             << std::abs(matlab[case_index][metric] - cpp[metric]) << ','
             << (validity_pass ? 1 : 0)
             << ",actual_MATLAB_batch_NLP1_equations_2026-11-07\n";
    }
  }
  return true;
}

}  // namespace optimizer
