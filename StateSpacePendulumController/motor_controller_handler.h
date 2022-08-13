#ifndef MOTOR_CONTROLLER_HANDLER_HEADER
#define MOTOR_CONTROLLER_HANDLER_HEADER

/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

/****************************************************************************************
 *                   P U B L I C   F U N C T I O N   D E C L A R A T I O N S 
 ****************************************************************************************/

void motor_controller_handler_init();

void motor_controller_handler_set_current(float current_command);

/**
* @brief      Gets the motor speed from the bldc controller by sending
*             the UART request and then parsing the read bytes.
*             
*             Upon request the bldc controller will send 4 bytes following this structure:
*             [0] = sign character, '+' or '-'
*             [1] = bits [6-0] are the most significant bits of the 14 bit unsigned speed data
*             [2] = bits [6-0] are the least significant bits of the 14 bit unsigned speed data
*             [4] = New line character to indicate end of transmission
*
* @return     A float containing the motor speed in RPMs
*/
float motor_controller_handler_get_speed();

#endif //MOTOR_CONTROLLER_HANDLER_HEADER
