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
 
#define ANGLE_SAMPLING_TIME_US 10000
//#define ANGLE_SAMPLING_TIME_US 25000
//#define ANGLE_SAMPLING_TIME_US 5000

/****************************************************************************************
 *                              G L O B A L   V A R I A B L E S
 ****************************************************************************************/
 
struct pendulum_controller_state_S pendulum_controller_state = {0};
struct pendulum_sensing_vars_S pendulum_sensing_vars = {0};

// Control ISR timer
hw_timer_t * sampling_timer = NULL;
volatile uint8_t update_sampling;

/****************************************************************************************
 *                               I S R   C A L L B A C K S                      
 ****************************************************************************************/

void IRAM_ATTR sampling_isr() {
  update_sampling = 1;
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

  // Set sampling trigger to ANGLE_SAMPLING_TIME_US microseconds
  update_sampling = 0;
  sampling_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(sampling_timer, &sampling_isr, true);
  timerAlarmWrite(sampling_timer, ANGLE_SAMPLING_TIME_US, true);
  timerAlarmEnable(sampling_timer);

}

void loop() {

  if (update_sampling)
  {
    // TODO: Potentially move the controls code to the ISR callback itself and just leave the prints here
    pendulum_sensing_vars.pendulum_angle = pendulum_sensing_get_adjusted_angle();
    pendulum_sensing_vars.pendulum_speed = pendulum_sensing_get_angular_speed(pendulum_sensing_vars.pendulum_angle, 1000000.0f/ANGLE_SAMPLING_TIME_US);
    pendulum_sensing_vars.motor_speed = motor_controller_handler_get_speed();

    // TODO: fix LPF so that it actually makes sense
    pendulum_sensing_vars.pendulum_angle_filtered = 0.7f * pendulum_sensing_vars.pendulum_angle_filtered + 0.3f * pendulum_sensing_vars.pendulum_angle;
    pendulum_sensing_vars.pendulum_speed_filtered = 0.7f * pendulum_sensing_vars.pendulum_speed_filtered + 0.3f * pendulum_sensing_vars.pendulum_speed;
    pendulum_sensing_vars.motor_speed_filtered = 0.7f * pendulum_sensing_vars.motor_speed_filtered + 0.3f * pendulum_sensing_vars.motor_speed;

    pendulum_controller(&pendulum_sensing_vars, &pendulum_controller_state);

    motor_controller_handler_set_current(pendulum_controller_state.control_law);

    //Serial.print(lpf_angle);
    //Serial.print(sp);
    //Serial.print(" ");
    Serial.print("180 -180 ");
    Serial.println(pendulum_sensing_vars.pendulum_angle_filtered);
    //Serial.print(w);
    //Serial.print(" ");
    //Serial.println(lpf_w);
    //Serial.println(motor_speed);
    //Serial.print(" ");
    //Serial.println(control_law);
    
    update_sampling = 0;
  }

}
