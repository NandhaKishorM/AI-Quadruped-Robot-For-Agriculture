#include "spot_micro_transition_idle.h"

#include "spot_micro_idle.h"
#include "spot_micro_motion_cmd.h"
#include "spot_micro_state.h"

SpotMicroTransitionIdleState::SpotMicroTransitionIdleState() {
  // Construcotr, doesn't need to do anything, for now...
  //std::cout << "SpotMicroTransitionIdleState Ctor" << std::endl;
}

SpotMicroTransitionIdleState::~SpotMicroTransitionIdleState() {
  //std::cout << "SpotMicroTransitionIdleState Dtor" << std::endl;
}


void SpotMicroTransitionIdleState::init(const smk::BodyState& body_state,
                                         const SpotMicroNodeConfig& smnc,
                                         const Command& cmd,
                                         SpotMicroMotionCmd* smmc) {
  // Set initial state and end state
  // Get starting body state
  start_body_state_ = body_state;
 
  // Create end state 
  // Create end state feet positions, a default foot stance
  end_body_state_.leg_feet_pos = smmc->getLieDownStance();

  // End body state position and angles
  end_body_state_.euler_angs.phi = 0.0f;
  end_body_state_.euler_angs.theta = 0.0f;
  end_body_state_.euler_angs.psi = 0.0f;

  end_body_state_.xyz_pos.x = 0.0f;
  end_body_state_.xyz_pos.y = smnc.lie_down_height;
  end_body_state_.xyz_pos.z = 0.0f;

  // Initialize filters
  float dt = smnc.dt;
  float tau = smnc.transit_tau;
  float rl = smnc.transit_rl;
  float rl_ang = smnc.transit_angle_rl;

  initBodyStateFilters(dt, tau, rl, rl_ang,
                       body_state, &body_state_filters_);

  // Set destination commands for all filters
  setBodyStateFilterCommands(end_body_state_, &body_state_filters_);
}


void SpotMicroTransitionIdleState::handleInputCommands(
                                   const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc,
                                   smk:: BodyState* body_state_cmd) {
  if (smnc.debug_mode) {
    std::cout << "In Spot Micro Transition Idle State" << std::endl;
  }
  
  // Check if desired end state reached, if so, change to stand state
  if (checkBodyStateEquality(body_state, end_body_state_, 0.001f)) {
    changeState(smmc, std::make_unique<SpotMicroIdleState>());
  
  } else {
    // Otherwise, rise filters and assign output values to body state command
    runFilters(&body_state_filters_);

    // Assing filter values to cmd
    assignFilterValuesToBodyState(body_state_filters_,
                                  body_state_cmd);

    // Send command
    smmc->setServoCommandMessageData();
    smmc->publishServoProportionalCommand();

  }

}



