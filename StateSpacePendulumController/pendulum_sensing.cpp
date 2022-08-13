/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

#include <Arduino.h>
#include "pendulum_sensing.h"
#include <SPI.h>

/****************************************************************************************
 *                                       D E F I N E S
 ****************************************************************************************/

#define SPI_BUS_FREQUENCY_HZ (8000000)

#define ANGLE_FULL_SCALE_DEG (360.0f)
#define ANGLE_DATA_LENGTH_BITS (16)
#define ANGLE_RESOLUTION_DEG_PER_BIT ( ANGLE_FULL_SCALE_DEG/((float)(2<<(ANGLE_DATA_LENGTH_BITS-1)) ) )

#define INITIAL_ANGLE_DEG (77.45f)

/****************************************************************************************
 *                       P R I V A T E   D A T A   D E F I N I T I O N S
 ****************************************************************************************/

// SPI bus objects
SPISettings SPI_bus_config = SPISettings(SPI_BUS_FREQUENCY_HZ, MSBFIRST, SPI_MODE0);
SPIClass* pendulum_sensor_SPI_handle = new SPIClass(VSPI);

/****************************************************************************************
 *                               P R I V A T E   F U N C T I O N S                 
 ****************************************************************************************/

/**
 * @brief      Reads the uint16 raw angle fomr the pendulum sensor through SPI
 *
 * @return     Raw uint16 pendulum angle
 */
static uint16_t pendulum_sensing_read_raw_angle_UI16()
{
  // Start SPI transmission. Uses the SPI settings object for correct mode operation
  digitalWrite(SS, LOW);
  pendulum_sensor_SPI_handle->beginTransaction(SPI_bus_config);

  // Read 16-bit unsigned angle
  const uint16_t angle = pendulum_sensor_SPI_handle->transfer16(0x0000);

  // End SPI transmission
  pendulum_sensor_SPI_handle->endTransaction();
  digitalWrite(SS, HIGH);

  return angle;
}

/****************************************************************************************
 *                                  P U B L I C   F U N C T I O N S
 ****************************************************************************************/

void pendulum_sensing_start_bus()
{
  // Starts the SPI bus
  pendulum_sensor_SPI_handle->begin();
  pinMode(SS, OUTPUT);
}

float pendulum_sensing_get_adjusted_angle()
{
  const uint16_t raw_angle = pendulum_sensing_read_raw_angle_UI16();

  // Get the original measured angle in degrees
  const float original_angle = raw_angle * ANGLE_RESOLUTION_DEG_PER_BIT;

  // Adjust the original angle to measure 180 degrees when the pendulum is hanging free
  // Do this by removing its original bias (INITIAL_ANGLE_DEG) and then add 180
  const float absolute_angle = original_angle - INITIAL_ANGLE_DEG + 180.0f ;
  
  //int divisor = floor(angle / 360.0f);
  //angle = angle - divisor*360;
  //const float wrapped_angle = absolute_angle % 360.0f;
  // TODO: Delete comments once proved

  // Float modulo operation to wrap angle in 360 deg range
  const float wrapped_angle_360 = absolute_angle - (trunc(absolute_angle/360.0f) * 360.0f);
  
  // Shift wrap to -180 to 180 range
  float wrapped_angle_180 = wrapped_angle_360;
  if(wrapped_angle_360 > 180)
  {
    wrapped_angle_180 = wrapped_angle_360 - 360.0f;
  }
  
  return wrapped_angle_180;
}


float pendulum_sensing_get_angular_speed(const float angle, const float time_step)
{
  static float previousAngle = 0;
  
  // Since the pendulum measures 180 deg when hanging free, there is a chance of
  // agressive jumps between 180 and -180 degs. The following logig helps with the
  // derivative around this point. Updated angle holds a temporal angle based on this.
  float updated_angle = angle;
  
  // Check if there was a change of sign around 180 degs
  if ( abs(angle) > 150 && angle*previousAngle < 0 )
  {
    // Update the angle to a range of 0 to 360
    updated_angle = 360.0f - abs(angle);

    // Once the updated angle is in a range that allows differentiation, apply the same
    // sign as the previous measurement
    if(previousAngle < 0.0f)
    {
      updated_angle *= -1.0f;
    }
  }

  // Get the difference based on the updated angle and the previous measurement
  const float angle_difference = updated_angle - previousAngle;

  // Keep track of measurement for next iteration
  previousAngle = angle;

  // Return derivative
  return angle_difference/time_step;
}
