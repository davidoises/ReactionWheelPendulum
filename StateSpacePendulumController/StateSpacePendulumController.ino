/*
 * @file StateSpacePendulumController
 * 
 * Zero regulation, state space controller of the angular position of reaction wheel pendulum.
 * Intended to regulate the pendulum angular positino to 0 degrees, but an utomatic setpoint
 * layer allows stability in the presence of mechanical unbalance in the pendulum.
 *
 * @author [davidoises]
 * @version  V2.0
 * @date  2022-June-10
 * @https://github.com/davidoises/ReactionWheelPendulum
 */
 
/****************************************************************************************
 *                                       I N C L U D E S
 ****************************************************************************************/

#include "src/drivers/motor_controller_handler.h"
#include "src/drivers/pendulum_sensing.h"
#include "src/controls/pendulum_controller.h"

/****************************************************************************************
 *                                        D E F I N E S
 ****************************************************************************************/
 
#define CONTROL_ISR_PERIOD_US 10000
#define CONTROL_ISR_FREQUENCY_HZ (1000000.0f/CONTROL_ISR_PERIOD_US)

// 7 HZ Low pass filter constant
#define SENSING_LPF_CUTOFF_HZ (7.0f)
#define SENSING_LPF_ALPHA (1.0f / (1.0f + CONTROL_ISR_FREQUENCY_HZ*(1.0f/(2*PI*SENSING_LPF_CUTOFF_HZ))))

/****************************************************************************************
 *                              G L O B A L   V A R I A B L E S
 ****************************************************************************************/
 
struct pendulum_controller_state_S pendulum_controller_state = {0};
struct pendulum_sensing_vars_S pendulum_sensing_vars = {0};

// Control ISR timer
hw_timer_t * sampling_timer = NULL;

// Flag to run code at 100Hz in main loop
volatile uint8_t tick_100Hz;

/****************************************************************************************
 *                               I S R   C A L L B A C K S                      
 ****************************************************************************************/

void IRAM_ATTR sampling_isr() {
  // Trigger a flag for other code to run at 100Hz
  tick_100Hz = 1;
}

/****************************************************************************************
 *                        A R D U I N O   B A S E   F U N C T I O N S                  
 ****************************************************************************************/

void setup()
{
  // Start primary serial for user communications  
  Serial.begin(115200);

  // User guard to prevent unintended/immediate oepration when booting
  Serial.println("Press anything to start execution");
  while(Serial.available() == 0) {}

  // Print some warning for the user
  Serial.println("Starting program in 4 second");

  // Initialize SPI bus for pendulum angle sensor
  pendulum_sensing_start_bus();

  // Initializes motor controller
  motor_controller_handler_init();

  delay(4000);

  // Set sampling trigger to CONTROL_ISR_PERIOD_US microseconds
  tick_100Hz = 0;
  sampling_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(sampling_timer, &sampling_isr, true);
  timerAlarmWrite(sampling_timer, CONTROL_ISR_PERIOD_US, true);
  timerAlarmEnable(sampling_timer);

}

void loop()
{
  // Code running at 100Hz
  if (tick_100Hz)
  {
    // Controls code was to slow to run directly in the ISR so moved to main loop
    // The slowliness is due to the UART communication for the motor controller.
    
    // 1. Gather sensgin variables
    pendulum_sensing_vars.pendulum_angle = pendulum_sensing_get_adjusted_angle();
    pendulum_sensing_vars.pendulum_speed = pendulum_sensing_get_angular_speed(pendulum_sensing_vars.pendulum_angle, 1.0f/CONTROL_ISR_FREQUENCY_HZ);
    pendulum_sensing_vars.motor_speed = motor_controller_handler_get_speed();
    
    // Low pass filter mesurements from sensors
    pendulum_sensing_vars.pendulum_angle_filtered = (1.0f-SENSING_LPF_ALPHA) * pendulum_sensing_vars.pendulum_angle_filtered + SENSING_LPF_ALPHA * pendulum_sensing_vars.pendulum_angle;
    pendulum_sensing_vars.pendulum_speed_filtered = (1.0f-SENSING_LPF_ALPHA) * pendulum_sensing_vars.pendulum_speed_filtered + SENSING_LPF_ALPHA * pendulum_sensing_vars.pendulum_speed;
    pendulum_sensing_vars.motor_speed_filtered = (1.0f-SENSING_LPF_ALPHA) * pendulum_sensing_vars.motor_speed_filtered + SENSING_LPF_ALPHA * pendulum_sensing_vars.motor_speed;
    
    // 2. Run the control loops
    pendulum_controller(&pendulum_sensing_vars, &pendulum_controller_state);
    
    // 3. Send the actual command to the actuator
    motor_controller_handler_set_current(pendulum_controller_state.control_law);

    // Data for serial plotter
    Serial.print("180 -180 ");
    Serial.println(pendulum_sensing_vars.pendulum_angle_filtered);
    //Serial.print(" ");
    //Serial.println(pendulum_sensing_vars.pendulum_speed_filtered);
    //Serial.print(" ");
    //Serial.println(pendulum_sensing_vars.motor_speed_filtered);
    
    tick_100Hz = 0;
  }
}
