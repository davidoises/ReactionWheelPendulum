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
 *                              G L O B A L   V A R I A B L E S
 ****************************************************************************************/

float lpf_angle = 0;
float hlpf_angle = 0;
float lpf_w = 0;
float lpf_mot = 0;
float sp = 0;
float lpf_sp = 0;
float control_law = 0;

uint8_t swingged_up = 0;

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
    float angle = pendulum_sensing_get_adjusted_angle();
    float w = pendulum_sensing_get_angular_speed(angle, 1000000.0f/ANGLE_SAMPLING_TIME_US);
    float motor_speed = motor_controller_handler_get_speed();

    lpf_angle = 0.7*lpf_angle + 0.3*angle;
    hlpf_angle = 0.9*hlpf_angle + 0.1*angle;
    lpf_w = 0.7*lpf_w + 0.3*w;
    lpf_mot = 0.7*lpf_mot + 0.3*motor_speed;

    if(abs(angle) < 27.0)
    {
      
      float kp = 240.0;
      float kd = 40.0;
      float ks = 0.001f*6.0f; // updated since the scaling used to be wrong
      float kt = 0.0075;

      if(abs(angle) < 6.0)
      {
        float error = lpf_angle - sp;
        sp = sp - kt*error;
        //lpf_sp = 0.7*lpf_sp + 0.3*sp;
        lpf_sp = sp;
      }
      else
      {
        sp = 0;
      }
  
      control_law = kp*(lpf_angle - lpf_sp) + kd*lpf_w - ks*lpf_mot;

      
      /* End of control law calculation */
  
      /* Start control law conditioning*/
      control_law = constrain(control_law, -5000, 5000);
  
      // linerization method
      float mapped_control = map(abs(control_law), 0, 5000, 600, 5000);
      if(control_law < 0)
      {
        control_law = -1*mapped_control;
      }
      else
      {
        control_law = mapped_control;
      }
    }
    else
    {
      if(swingged_up == 0)
      {
        //control_law = 0;
        if(lpf_w < 0)
        {
          control_law = 3300*0.4;
        }
        else
        {
          control_law = -3300*0.4;
        }
      }
      else
      {
        control_law = 0;
      }
      
    }

    //Serial.print(lpf_angle);
    //Serial.print(sp);
    //Serial.print(" ");
    Serial.print("180 -180 ");
    Serial.println(lpf_angle);
    //Serial.print(w);
    //Serial.print(" ");
    //Serial.println(lpf_w);
    //Serial.println(motor_speed);
    //Serial.print(" ");
    //Serial.println(control_law);
    
    if(abs(angle) < 10.0)
    {
      swingged_up = 1;
      //swingged_up = 0;
    }
    
    /* End control law conditioning*/

    motor_controller_handler_set_current(control_law);
    
    update_sampling = 0;
  }

}

/****************************************************************************************
 *                               P R I V A T E   F U N C T I O N S                 
 ****************************************************************************************/
