#ifndef MOTOR_CONTROLLER_HANDLER_HEADER
#define MOTOR_CONTROLLER_HANDLER_HEADER

/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

/****************************************************************************************
 *                   P U B L I C   F U N C T I O N   D E C L A R A T I O N S 
 ****************************************************************************************/

/**
 * @brief      Initializes a serial port to send commands to motor controller.
 *             Sends the initial commands to enable and start motor operation.
 */
void motor_controller_handler_init();

/**
 * @brief      Sens the current command over UART to the motor controller following this
 *             6 bytes structure:
 *             [0] = ':' to indicate the start of a command
 *             [1] = 'c' to indicate a current command for the motor
 *             [2] = sign character, '+' or '-'
 *             [3] = bits [6-0] are the most significant bits of the 14 bit unsigned current data
 *             [4] = bits [6-0] are the least significant bits of the 14 bit unsigned current data
 *             [5] = new line character to indicate end of command
 *
 * @param[in]  current_command  The current reference for the motor control loop
 */
void motor_controller_handler_set_current(const float current_command);

/**
* @brief      Gets the motor speed from the bldc controller by sending
*             the UART request and then parsing the read bytes.
*             
*             The controller expects a 3 byte request as follows:
*             [0] = ':' to indicate the start of a command
*             [1] = 'g' to indicate a "get speed" command
*             [2] = new line character to indicate end of command
*             
*             Upon request the bldc controller will send 4 bytes following this structure:
*             [0] = sign character, '+' or '-'
*             [1] = bits [6-0] are the most significant bits of the 14 bit unsigned speed data
*             [2] = bits [6-0] are the least significant bits of the 14 bit unsigned speed data
*             [3] = new line character to indicate end of response
*
* @return     A float containing the motor speed in RPMs
*/
float motor_controller_handler_get_speed();

#endif //MOTOR_CONTROLLER_HANDLER_HEADER
