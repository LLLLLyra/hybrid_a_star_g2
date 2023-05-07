
#include "node3d.h"

#include "glog/logging.h"

Node3d::Node3d(double x, double y, double phi) {
  x_ = x;
  y_ = y;
  phi_ = phi;
}

Node3d::Node3d(const double x, const double y, const double phi,
               const double kappa) {
  x_ = x;
  y_ = y;
  phi_ = phi;
  kappa_ = kappa;
}

Node3d::Node3d(double x, double y, double phi, const double kappa,
               const std::vector<double>& XYbounds,
               const planing::PlannerOpenSpaceConfig& open_space_conf) {
  CHECK_EQ(XYbounds.size(), 4U)
      << "XYbounds size is not 4, but" << XYbounds.size();

  x_ = x;
  y_ = y;
  phi_ = phi;
  kappa_ = kappa;

  x_grid_ = static_cast<int>(
      (x_ - XYbounds[0]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  y_grid_ = static_cast<int>(
      (y_ - XYbounds[2]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  phi_grid_ = static_cast<int>(
      (phi_ - (-M_PI)) /
      open_space_conf.warm_start_config().phi_grid_resolution());

  traversed_x_.push_back(x);
  traversed_y_.push_back(y);
  traversed_phi_.push_back(phi);

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
}

Node3d::Node3d(double x, double y, double phi,
               const std::vector<double>& XYbounds,
               const planing::PlannerOpenSpaceConfig& open_space_conf) {
  CHECK_EQ(XYbounds.size(), 4U)
      << "XYbounds size is not 4, but" << XYbounds.size();

  x_ = x;
  y_ = y;
  phi_ = phi;

  x_grid_ = static_cast<int>(
      (x_ - XYbounds[0]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  y_grid_ = static_cast<int>(
      (y_ - XYbounds[2]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  phi_grid_ = static_cast<int>(
      (phi_ - (-M_PI)) /
      open_space_conf.warm_start_config().phi_grid_resolution());

  traversed_x_.push_back(x);
  traversed_y_.push_back(y);
  traversed_phi_.push_back(phi);

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
}

Node3d::Node3d(const std::vector<double>& traversed_x,
               const std::vector<double>& traversed_y,
               const std::vector<double>& traversed_phi,
               const std::vector<double>& XYbounds,
               const planing::PlannerOpenSpaceConfig& open_space_conf) {
  CHECK_EQ(XYbounds.size(), 4U)
      << "XYbounds size is not 4, but" << XYbounds.size();
  CHECK_EQ(traversed_x.size(), traversed_y.size());
  CHECK_EQ(traversed_x.size(), traversed_phi.size());

  x_ = traversed_x.back();
  y_ = traversed_y.back();
  phi_ = traversed_phi.back();

  // XYbounds in xmin, xmax, ymin, ymax
  x_grid_ = static_cast<int>(
      (x_ - XYbounds[0]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  y_grid_ = static_cast<int>(
      (y_ - XYbounds[2]) /
      open_space_conf.warm_start_config().xy_grid_resolution());
  phi_grid_ = static_cast<int>(
      (phi_ - (-M_PI)) /
      open_space_conf.warm_start_config().phi_grid_resolution());

  traversed_x_ = traversed_x;
  traversed_y_ = traversed_y;
  traversed_phi_ = traversed_phi;

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
  step_size_ = traversed_x.size();
}

Box2d Node3d::GetBoundingBox(const vehicle::VehicleParam& vehicle_param_,
                             const double x, const double y, const double phi) {
  double ego_length = vehicle_param_.length();
  double ego_width = vehicle_param_.width();
  double shift_distance =
      ego_length / 2.0 - vehicle_param_.back_edge_to_center();
  Box2d ego_box(
      {x + shift_distance * std::cos(phi), y + shift_distance * std::sin(phi)},
      phi, ego_length, ego_width);
  return ego_box;
}

bool Node3d::operator==(const Node3d& right) const {
  return right.GetIndex() == index_;
}

std::string Node3d::ComputeStringIndex(int x_grid, int y_grid, int phi_grid) {
  return std::to_string(x_grid) + "_" + std::to_string(y_grid) + "_" +
         std::to_string(phi_grid);
}
