#ifndef REED_SHEPP_PATH_G2_H_
#define REED_SHEPP_PATH_G2_H_

#include "reeds_shepp_path.h"
#include "steering_functions/hc_cc_state_space/cc00_reeds_shepp_state_space.hpp"

/**
 * @brief Reed Shepp curves with G2 continuity
 * ref: https://github.com/hbanzhaf/steering_functions
 *
 */
class ReedSheppG2 : public ReedShepp {
 public:
  ReedSheppG2(const vehicle::VehicleParam& vehicle_param,
              const planing::PlannerOpenSpaceConfig& open_space_conf);

  /**
   * @brief find the shortest RS path with G2 continuity
   *
   * @param start_node
   * @param end_node
   * @param optimal_path
   * @return true
   * @return false
   */
  bool ShortestRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   std::shared_ptr<ReedSheppPath> optimal_path) override;

 protected:
  void StateTrans(const std::shared_ptr<Node3d> node, steering::State* state);
  void PathTrans(std::vector<steering::State>& path,
                 std::shared_ptr<ReedSheppPath> optimal_path);

 protected:
  std::shared_ptr<steering::CC00_Reeds_Shepp_State_Space> state_space_ =
      nullptr;

  double discretization_;
};

#endif  // REED_SHEPP_PATH_G2_H_