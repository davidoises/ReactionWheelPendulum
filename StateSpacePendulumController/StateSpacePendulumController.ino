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

#include "motor_controller_handler.h"
#include "pendulum_sensing.h"

/****************************************************************************************
 *                                        D E F I N E S
 ****************************************************************************************/
 
#define ANGLE_SAMPLING_TIME_US 10000
//#define ANGLE_SAMPLING_TIME_US 25000
//#define ANGLE_SAMPLING_TIME_US 5000

/****************************************************************************************
 *                                          T Y P E S
 ****************************************************************************************/

struct pendulum_controller_state_S
{
  float control_law;        // Current command coming out of the controller
  bool  at_operating_point; // Flag indicating whether the pendulum needs to be swung up to the operating region
  float adjusted_reference; // Reference for the pendulum position, automatically set for optimum control
};

struct pendulum_sensing_vars_S
{
  float pendulum_angle;
  float pendulum_speed;
  float motor_speed;

  float pendulum_angle_filtered;
  float pendulum_speed_filtered;
  float motor_speed_filtered;
};

/****************************************************************************************
 *                              G L O B A L   V A R I A B L E S
 ****************************************************************************************/
 
struct pendulum_controller_state_S pendulum_controller_state = {0};
struct pendulum_sensing_vars_S pendulum_sensing_vars = {0};

// Sampling timer
hw_timer_t * sampling_timer = NULL;
volatile uint8_t update_sampling;

/****************************************************************************************
 *                               I S R   C A L L B A C K S                      
 ****************************************************************************************/

void IRAM_ATTR sampling_isr() {
  update_sampling = 1;
}

/****************************************************************************************
 *                P R I V A T E   F U N C T I O N   D E C L A R A T I O N S                  
 ****************************************************************************************/

void pendulum_controller(struct pendulum_sensing_vars_S* sensing_vars, struct pendulum_controller_state_S* controller_state);
float pendulum_reference_correction(const float pendulum_angle, const float pendulum_reference);
float pendulum_position_control(const float pendulum_angle, const float pendulum_speed, const float motor_speed, const float pendulum_reference);
float pendulum_swing_up(const bool at_operating_point, const float pendulum_speed);

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
    pendulum_sensing_vars.pendulum_angle = pendulum_sensing_get_adjusted_angle();
    pendulum_sensing_vars.pendulum_speed = pendulum_sensing_get_angular_speed(pendulum_sensing_vars.pendulum_angle, 1000000.0f/ANGLE_SAMPLING_TIME_US);
    pendulum_sensing_vars.motor_speed = motor_controller_handler_get_speed();

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

/****************************************************************************************
 *                               P R I V A T E   F U N C T I O N S                 
 ****************************************************************************************/

void pendulum_controller(struct pendulum_sensing_vars_S* sensing_vars, struct pendulum_controller_state_S* controller_state)
{
  
  if(abs(sensing_vars->pendulum_angle) < 27.0)
  {
    
    if(abs(sensing_vars->pendulum_angle) < 6.0f)
    {
      controller_state->adjusted_reference = pendulum_reference_correction(sensing_vars->pendulum_angle_filtered, controller_state->adjusted_reference);
    }
    else
    {
      controller_state->adjusted_reference = 0.0f;
    }
    
    controller_state->control_law = pendulum_position_control(sensing_vars->pendulum_angle_filtered, sensing_vars->pendulum_speed_filtered, sensing_vars->motor_speed_filtered, controller_state->adjusted_reference);
  }
  else
  {
    controller_state->control_law = pendulum_swing_up(controller_state->at_operating_point, sensing_vars->pendulum_speed_filtered);
  }

  if(abs(sensing_vars->pendulum_angle) < 10.0)
  {
    controller_state->at_operating_point = true;
  }
}

float pendulum_reference_correction(const float pendulum_angle, const float pendulum_reference)
{
  float kt = 0.0075;

  const float error = pendulum_angle - pendulum_reference;
  const float new_reference = pendulum_reference - kt*error;

  return new_reference;
}

float pendulum_position_control(const float pendulum_angle, const float pendulum_speed, const float motor_speed, const float pendulum_reference)
{
  float kp = 240.0;
  float kd = 40.0;
  float ks = 0.001f*6.0f; // updated since the scaling used to be wrong  

  float motor_current_command = kp*(pendulum_angle - pendulum_reference) + kd*pendulum_speed - ks*motor_speed;

  motor_current_command = constrain(motor_current_command, -5000, 5000);

  // linerization method
  float mapped_control = map(abs(motor_current_command), 0, 5000, 600, 5000);
  if(motor_current_command < 0)
  {
    motor_current_command = -1*mapped_control;
  }
  else
  {
    motor_current_command = mapped_control;
  }

  return motor_current_command;
}

float pendulum_swing_up(const bool at_operating_point, const float pendulum_speed)
{
  float motor_current_command = 0;
  
  if(!at_operating_point)
  {
    if(pendulum_speed < 0.0f)
    {
      motor_current_command = 3300.0f*0.4f;
    }
    else
    {
      motor_current_command = -3300.0f*0.4f;
    }
  }

  return motor_current_command;
}
