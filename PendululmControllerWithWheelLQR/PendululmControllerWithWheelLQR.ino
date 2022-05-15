#include <SPI.h>

#define RXD1 9
#define TXD1 10

#define ANGLE_CONVERTION 360.0/65536.0
#define INITIAL_ANGLE 77.45
#define ANGLE_SAMPLING_TIME_US 10000
//#define ANGLE_SAMPLING_TIME_US 25000
//#define ANGLE_SAMPLING_TIME_US 5000

uint16_t readAngleRaw16();

SPISettings s = SPISettings(8000000, MSBFIRST, SPI_MODE0);
SPIClass* vspi = new SPIClass(VSPI);

// Sampling timer
hw_timer_t * sampling_timer = NULL;
volatile uint8_t update_sampling;
void IRAM_ATTR sampling_isr() {
  update_sampling = 1;
}

float lpf_angle = 0;
float hlpf_angle = 0;
float lpf_w = 0;
float lpf_mot = 0;
float sp = 0;
float lpf_sp = 0;
float control_law = 0;

uint8_t swingged_up = 0;

void setup() {

  Serial.begin(115200);
  //Serial2.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD1, TXD1); //Use different UART pins

  Serial.println("Press anything to start execution");
  while(Serial.available() == 0) {
    }

  Serial.println("Starting program in 4 second");

  // Initialize SPI
  vspi->begin();
  pinMode(SS, OUTPUT);

  // Initialize motor
  Serial2.print(":");
  Serial2.print("e");
  Serial2.print('\n');

  Serial2.print(":");
  Serial2.print("r");
  Serial2.print('\n');

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
    uint16_t rawAngle = readAngleRaw16();
    float angle = getWrappedAngle(rawAngle);
    float w = getAngularSpeed(angle);
    int motor_speed = getMotorSpeed();

    lpf_angle = 0.7*lpf_angle + 0.3*angle;
    hlpf_angle = 0.9*hlpf_angle + 0.1*angle;
    lpf_w = 0.7*lpf_w + 0.3*w;
    lpf_mot = 0.7*lpf_mot + 0.3*( (float) motor_speed );

    if(abs(angle) < 27.0)
    {
      
      float kp = 240.0;
      float kd = 40.0;
      float ks = 0.001;
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

    /* Start of motor command transmission*/
    
    //  range of 5000mA in 14 bits
    uint16_t motor_current = (abs(control_law)*1000)/(153*2);
    uint8_t MSB = ( (motor_current >> 7) & 0x00FF) | 0x80; // set MSB bit to one always, test on other microncontroller without this hack
    uint8_t LSB = (motor_current & 0x00FF) | 0x80;

    Serial2.print(":");
    Serial2.print("c");
    if(control_law >= 0)
    {
      Serial2.print("+");
    }
    else
    {
      Serial2.print("-");
    }
    Serial2.write(MSB);
    Serial2.write(LSB);
    Serial2.print('\n');

    /* Ending of motor command transmission*/
    
    update_sampling = 0;
  }

}

int getMotorSpeed()
{
  Serial2.print(":");
  Serial2.print("g");
  Serial2.print('\n');

  char received_data[4];
  int read_length = Serial2.readBytesUntil('\n', received_data, 4);
  if(read_length == 3)
  {
    int speed_data = ((received_data[1] & 0x7F) << 7) | (received_data[2] & 0x7F);
    speed_data *= 2;
    
    if(received_data[0] == '-')
    {
      speed_data *= -1;
    }
    return speed_data;
    //Serial.println(speed_data);
  }
  //else
  //{
  //  Serial.println(read_length);
  //}

  return 0;
}

uint16_t readAngleRaw16(){

    digitalWrite(SS, LOW);
    vspi->beginTransaction(s);

    //angle = SPI.transfer16(0x0000); //Read 16-bit angle
    uint16_t angle = vspi->transfer16(0x0000);
    
    vspi->endTransaction();
    digitalWrite(SS, HIGH);

    return angle;
}

// gets the encoder agulum in range [-180, 180]
float getWrappedAngle(uint16_t rawAngle)
{
  float angle = rawAngle*ANGLE_CONVERTION + (180.0 - INITIAL_ANGLE);
  int divisor = floor(angle / 360.0);
  angle = angle - divisor*360;

  if(angle > 180)
  {
    angle = angle - 360.0;
  }
  
  return angle;
}

// gets the angular speed
float getAngularSpeed(float angle)
{
  // Elapsed time calculation can be left out since timer is precisely sampling every 10ms
  static uint32_t previousT;
  uint32_t currentT = micros();
  float dt = (currentT - previousT);
  previousT = currentT;
  
  float originalAngle = angle;
  static float previousAngle;
  if ( abs(angle) > 150 && angle*previousAngle < 0 )
  {
    //Serial.println("Jump");
    angle = 360.0 - abs(angle);
    if(previousAngle < 0)
    {
      angle = angle * -1;
    }
  }
  float angleDiff = angle - previousAngle;
  previousAngle = originalAngle;
  
  return angleDiff*1000000.0/dt;
}
