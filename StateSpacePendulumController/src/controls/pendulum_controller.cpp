/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

#include <Arduino.h>
#include "pendulum_controller.h"

/****************************************************************************************
 *                                       D E F I N E S
 ****************************************************************************************/

 #define PENDULUM_REFERENCE_CORRECTION_K (0.0075f)

 #define PENDULUM_SS_CONTROLLER_PENDULUM_ANGLE_K (240.0f)
 #define PENDULUM_SS_CONTROLLER_PENDULUM_SPEED_K (40.0f)
 #define PENDULUM_SS_CONTROLLER_MOTOR_SPEED_K (0.001f*6.0f) // TODO: updated since the scaling used to be wrong

 #define PENDULUM_CONTROLLER_MAX_CURRENT_COMMAND_mA (5000.0f)
 #define PENDULUM_CONTROLLER_CURRENT_COMMAND_DEADZONE_mA (600.0f)

 #define PENDULUM_CONTROLLER_SWING_UP_CURRENT_COMMAND_mA (1320.0f)

/****************************************************************************************
 *                               P R I V A T E   F U N C T I O N S                 
 ****************************************************************************************/

/**
 * @brief      Adjusts the position reference for the pendulum controller with a proportional controller.
 *             The error just takes into consideration the angle of the pendulum and the previous reference.
 *
 * @param[in]  pendulum_angle      The pendulum angle
 * @param[in]  pendulum_reference  The previous pendulum reference
 *
 * @return     The new reference target for the pendulum position controller
 */
static float pendulum_reference_correction(const float pendulum_angle, const float pendulum_reference)
{

  const float error = pendulum_angle - pendulum_reference;
  const float new_reference = pendulum_reference - (PENDULUM_REFERENCE_CORRECTION_K * error);

  return new_reference;
}

/**
 * @brief      State space controller for a variable pendulum posistion reference coming from the
 *             automatic reference correction block.
 *             Adds a deadband due to motor controller deadzone due to friction/inertia.
 *
 * @param[in]  pendulum_angle      The pendulum angle
 * @param[in]  pendulum_speed      The pendulum speed
 * @param[in]  motor_speed         The motor speed
 * @param[in]  pendulum_reference  The pendulum position reference
 *
 * @return     The control law (motor command) to be sent to the actuator
 */
static float pendulum_position_control(const float pendulum_angle, const float pendulum_speed, const float motor_speed, const float pendulum_reference)
{

  // State space control law calculation. Motor command in mA
  float motor_current_command = PENDULUM_SS_CONTROLLER_PENDULUM_ANGLE_K*(pendulum_angle - pendulum_reference) + PENDULUM_SS_CONTROLLER_PENDULUM_SPEED_K*pendulum_speed - PENDULUM_SS_CONTROLLER_MOTOR_SPEED_K*motor_speed;

  // Saturate controller output within motor range of operation
  motor_current_command = constrain(motor_current_command, -PENDULUM_CONTROLLER_MAX_CURRENT_COMMAND_mA, PENDULUM_CONTROLLER_MAX_CURRENT_COMMAND_mA);

  // Adjust the output command with a linear mapping to account for motor controller deadzone
  float mapped_control = map(abs(motor_current_command), 0.0f, PENDULUM_CONTROLLER_MAX_CURRENT_COMMAND_mA, PENDULUM_CONTROLLER_CURRENT_COMMAND_DEADZONE_mA, PENDULUM_CONTROLLER_MAX_CURRENT_COMMAND_mA);
  if(motor_current_command < 0.0f)
  {
    motor_current_command = -1.0f*mapped_control;
  }
  else
  {
    motor_current_command = mapped_control;
  }

  return motor_current_command;
}

/**
 * @brief      Generates a motor command based on the speed of the pendulum to destabilize it.
 *
 * @param[in]  at_operating_point  Indicates whether the pendulum is in the desired region already
 * @param[in]  pendulum_speed      The pendulum speed
 *
 * @return     The motor command to destabilize the pendulum
 */
static float pendulum_swing_up(const bool at_operating_point, const float pendulum_speed)
{
  float motor_current_command = 0.0f;

  // TODO: This condition seems redundant again
  if(!at_operating_point)
  {
    // Swing up consists on commanding the actuator in the opposite direction of the pendulum speed.
    // This means destabilizing the pendulum with positive feedback as long as it is not in the operation region.
    if(pendulum_speed < 0.0f)
    {
      motor_current_command = PENDULUM_CONTROLLER_SWING_UP_CURRENT_COMMAND_mA;
    }
    else
    {
      motor_current_command = -PENDULUM_CONTROLLER_SWING_UP_CURRENT_COMMAND_mA;
    }
  }

  return motor_current_command;
}

/****************************************************************************************
 *                                  P U B L I C   F U N C T I O N S
 ****************************************************************************************/

void pendulum_controller(const struct pendulum_sensing_vars_S* sensing_vars, struct pendulum_controller_state_S* controller_state)
{

  // TODO: Use the filtered angle for these threshold checks
  // If pendulum is wihin linear region, run control loop
  if(abs(sensing_vars->pendulum_angle) < 27.0)
  {
    // When the pendulum angle is small enough, run the automatic reference correction to converge to stable reference
    if(abs(sensing_vars->pendulum_angle) < 6.0f)
    {
      controller_state->adjusted_reference = pendulum_reference_correction(sensing_vars->pendulum_angle_filtered, controller_state->adjusted_reference);
    }
    // Disable automatic reference correction when far from stable point to prevent extra swinging due to changes in reference
    else
    {
      controller_state->adjusted_reference = 0.0f;
    }

    // Calculate motor command based on state space controller
    controller_state->control_law = pendulum_position_control(sensing_vars->pendulum_angle_filtered, sensing_vars->pendulum_speed_filtered, sensing_vars->motor_speed_filtered, controller_state->adjusted_reference);
  }
  // If pendulum is outside of linear region, swing it until it gets back there
  else
  {
    controller_state->control_law = pendulum_swing_up(controller_state->at_operating_point, sensing_vars->pendulum_speed_filtered);
  }

  // TODO: This condition seems useles, put inside previous ifs
  if(abs(sensing_vars->pendulum_angle) < 10.0)
  {
    controller_state->at_operating_point = true;
  }
}
