/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

#include <Arduino.h>
#include "motor_controller_handler.h"

/****************************************************************************************
 *                                        D E F I N E S
 ****************************************************************************************/

// Note on UARTs:
//     3 UARTs can be mapped to any pin through internal mux.
//     These UARTs are tied to specific pins by default and using these pins can
//     provide some optimization (maybe not with arduino Serial library). Default pins:
//     UART0 RX:3 TX:1
//     UART1 RX:9 TX:10
//     UART2 RX16 TX:17
#define MOTOR_CONTROLLER_UART_RXD_PIN 9
#define MOTOR_CONTROLLER_UART_TXD_PIN 10

#define MOTOR_CONTROLLER_UART_BAUDRATE_bPS (115200)

#define MOTOR_SPEED_BUFFER_LENGTH (4U)

// Conversion factor for motor speed to bits
#define SPEED_FULL_SCALE_RPM (5461.0f)
#define SPEED_DATA_LENGTH_BITS (14)
#define SPEED_RESOLUTION_RPM_PER_BIT ( SPEED_FULL_SCALE_RPM/((float)(2<<(SPEED_DATA_LENGTH_BITS-1)) ) )

// Conversion factor for current command to bits
#define CURRENT_FULL_SCALE_mA (5000.0f)
#define CURRENT_DATA_LENGTH_BITS (14)
#define CURRENT_RESOLUTION_mA_PER_BIT ( CURRENT_FULL_SCALE_mA/( (float)(2<<(CURRENT_DATA_LENGTH_BITS-1)) ) )

/****************************************************************************************
 *                                  P U B L I C   F U N C T I O N S
 ****************************************************************************************/

void motor_controller_handler_init()
{
  // Start second serial for motor controller communications
  Serial2.begin(MOTOR_CONTROLLER_UART_BAUDRATE_bPS, SERIAL_8N1, MOTOR_CONTROLLER_UART_RXD_PIN, MOTOR_CONTROLLER_UART_TXD_PIN);
  
  // Arm motor controller (enable power stage)
  Serial2.print(":");
  Serial2.print("e");
  Serial2.print('\n');

  // Start operation of control loop
  Serial2.print(":");
  Serial2.print("r");
  Serial2.print('\n');
}

void motor_controller_handler_set_current(const float current_command)
{
  // Convert the current command into 14 bit usigned data
  const uint16_t current_command_bits = abs(current_command)/CURRENT_RESOLUTION_mA_PER_BIT;

  // Split the insgined data into most and leas significant 7-bit sections
  // 8th bit is set to 1 to deal with signal integrity issues
  const uint8_t current_command_MSB = ( (current_command_bits >> 7) & 0x00FF) | 0x80;
  const uint8_t current_command_LSB = (current_command_bits & 0x00FF) | 0x80;

  // Send the command over UART with the required structure
  Serial2.print(":");
  Serial2.print("c");
  if(current_command >= 0)
  {
    Serial2.print("+");
  }
  else
  {
    Serial2.print("-");
  }
  Serial2.write(current_command_MSB);
  Serial2.write(current_command_LSB);
  Serial2.print('\n');
}

float motor_controller_handler_get_speed()
{
  // Variable to return motor speed
  float motor_speed_rpm = 0;
  
  // bldc controller expects the string ":g\n" as speed data request
  Serial2.print(":");
  Serial2.print("g");
  Serial2.print('\n');

  // Array to store read data from the bldc controller
  char received_data[MOTOR_SPEED_BUFFER_LENGTH];

  // The readBytesUntil function will return the number of bytes read when:
  //     1. 4 bytes are received
  //     2. There is a timeout
  //     3. The specified termination character is received (termination not stored in buffer)
  // On a correct data transmission the buffer should only receive 3 bytes since
  // the readBytesUntil function wont store the new line character. Still the buffer
  // length is set to 4 to catch cases where a bad transmission ocurred and the 4th character was
  // not a new line character.
  const size_t read_length = Serial2.readBytesUntil('\n', received_data, MOTOR_SPEED_BUFFER_LENGTH);
  
  // Correct transmission should always be 3 bytes
  if(read_length == 3)
  {
    // Combine most and least significant bits into single uint16_t data
    const uint16_t motor_speed_abs_bits = ((received_data[1] & 0x7F) << 7) | (received_data[2] & 0x7F);

    // Apply resolution for conversion between bits and rpms
    const float motor_speed_abs_rpm = motor_speed_abs_bits * SPEED_RESOLUTION_RPM_PER_BIT;
    
    // Apply the correspoding sign to motor speed
    if(received_data[0] == '-')
    {
      motor_speed_rpm = motor_speed_abs_rpm * -1.0f;
    }
    else
    {
      motor_speed_rpm = motor_speed_abs_rpm;
    }
  }

  return motor_speed_rpm;
}
