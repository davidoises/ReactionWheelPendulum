#ifndef PENDULUM_CONTROLLER_HEADER
#define PENDULUM_CONTROLLER_HEADER

/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

/****************************************************************************************
 *                                          T Y P E S
 ****************************************************************************************/

struct pendulum_controller_state_S
{
  float control_law;        // Current command coming out of the controller
  bool  at_operating_point; // Flag indicating whether the reached the operating region. Never cleared to avoid continuous swing ups
  float adjusted_reference; // Reference for the pendulum position, automatically set for optimum control
};

struct pendulum_sensing_vars_S
{
  float pendulum_angle;           // Measured pendulum angle in degrees
  float pendulum_speed;           // Measured/estimated pendulum angular speed in degrees/s
  float motor_speed;              // Measured motor speed in RPM

  // Low pass filtered version of the measurements
  float pendulum_angle_filtered;
  float pendulum_speed_filtered;
  float motor_speed_filtered;
};

/****************************************************************************************
 *                   P U B L I C   F U N C T I O N   D E C L A R A T I O N S 
 ****************************************************************************************/

/**
 * @brief      Runs the state space controller to track the pendulum position reference.
 *             An automatic reference corrector adjusts the controllers reference to sensure 
 *             the pendulum reaches a stable point regardless of possible mechanical unbalances.
 *
 * @param[in]  sensing_vars      Pointer to sensing variables struct
 * @param      controller_state  Pointer to controller state struct
 */
void pendulum_controller(const struct pendulum_sensing_vars_S* sensing_vars, struct pendulum_controller_state_S* controller_state);

#endif //PENDULUM_CONTROLLER_HEADER
