#include "glog/logging.h"
#include "reed_shepp_path_g2.h"

ReedSheppG2::ReedSheppG2(const vehicle::VehicleParam& vehicle_param,
                         const planing::PlannerOpenSpaceConfig& open_space_conf)
    : ReedShepp(vehicle_param, open_space_conf) {
  discretization_ =
      planner_open_space_config_.warm_start_config().xy_grid_resolution();
  state_space_ = std::shared_ptr<steering::CC00_Reeds_Shepp_State_Space>(
      new steering::CC00_Reeds_Shepp_State_Space(max_kappa_, 1.0,
                                                 discretization_));
}

void ReedSheppG2::StateTrans(const std::shared_ptr<Node3d> node,
                             steering::State* state) {
  CHECK_NOTNULL(state);
  state->x = node->GetX();
  state->y = node->GetY();
  state->theta = node->GetPhi();
  state->kappa = node->GetKappa();
}

bool ReedSheppG2::ShortestRSP(const std::shared_ptr<Node3d> start_node,
                              const std::shared_ptr<Node3d> end_node,
                              std::shared_ptr<ReedSheppPath> optimal_path) {
  steering::State start_state, goal_state;
  StateTrans(start_node, &start_state);
  StateTrans(end_node, &goal_state);
  auto path = state_space_->get_path(start_state, goal_state);
  PathTrans(path, optimal_path);
  return true;
}

void ReedSheppG2::PathTrans(std::vector<steering::State>& path,
                            std::shared_ptr<ReedSheppPath> optimal_path) {
  optimal_path->x.clear();
  optimal_path->y.clear();
  optimal_path->phi.clear();
  optimal_path->gear.clear();

  for (const auto& p : path) {
    optimal_path->x.push_back(p.x);
    optimal_path->y.push_back(p.y);
    optimal_path->phi.push_back(p.theta);
    optimal_path->gear.push_back(p.d == 1.0);
  }
}