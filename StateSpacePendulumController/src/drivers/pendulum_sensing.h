#ifndef PENDULUM_SENSING_HEADER
#define PENDULUM_SENSING_HEADER

/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

/****************************************************************************************
 *                   P U B L I C   F U N C T I O N   D E C L A R A T I O N S 
 ****************************************************************************************/

/**
 * @brief      Starts the SPI bus for sensor communication
 */
void pendulum_sensing_start_bus();

/**
 * @brief      Gets the raw angle form the SPI sensor. Removes intial offset based on
 *             calibration tests and then wraps result to a range of -180 to 180 degs.
 *             The offset adjustment is done in a way that the angle will measure
 *             180 degs when the pendulum is hanging free (pointing to ground).
 *
 * @return     Pendulum angle in degrees in range of [-180, 180]
 */
float pendulum_sensing_get_adjusted_angle();

/**
 * @brief      Estimates the angular speed of the pendulum based on the angle measurements
 *             and the time step between measurements, i.e. discrete derivative.
 *             Deals with logic for derivative for jumps between -180 and 180 degs.
 *
 * @param[in]  angle      The pendulum measured angle
 * @param[in]  time_step  The time step between measurements
 *
 * @return     The pendulum angular speed
 */
float pendulum_sensing_get_angular_speed(const float angle, const float time_step);

#endif //PENDULUM_SENSING_HEADER
