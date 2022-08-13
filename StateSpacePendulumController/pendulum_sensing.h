#ifndef PENDULUM_SENSING_HEADER
#define PENDULUM_SENSING_HEADER

/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

/****************************************************************************************
 *                   P U B L I C   F U N C T I O N   D E C L A R A T I O N S 
 ****************************************************************************************/

void pendulum_sensing_start_bus();

// gets the encoder agulum in range [-180, 180]
/**
 * @brief      Gets the wrapped angle.
 *
 * @param[in]  rawAngle  The raw angle
 *
 * @return     The wrapped angle.
 */
float pendulum_sensing_get_adjusted_angle();

// gets the angular speed
/**
 * @brief      Gets the angular speed.
 *
 * @param[in]  angle  The angle
 *
 * @return     The angular speed.
 */
float pendulum_sensing_get_angular_speed(float angle);

#endif //PENDULUM_SENSING_HEADER
