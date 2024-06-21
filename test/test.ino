#include <MsTimer2.h>


// Front Motor Drive

#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4

#define MOTOR2_PWM 5
#define MOTOR2_ENA 6
#define MOTOR2_ENB 7

int f_speed = 0, r_speed = 0;
int front_motor_pwm = 0;
int rear_motor_pwm = 0;

void front_motor_control(int motor1_pwm)
{
   if (motor1_pwm > 0) // forward
  {
    digitalWrite(MOTOR1_ENA, HIGH);   // 방향은 설정에 따라 바꿀 것
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, motor1_pwm);
  }
  else if (motor1_pwm < 0) // backward
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, HIGH);
    analogWrite(MOTOR1_PWM, -motor1_pwm);
  }
  else
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, LOW);
    digitalWrite(MOTOR1_PWM, 0);
  }
}

void rear_motor_control(int motor2_pwm)
{
   if (motor2_pwm > 0) // forward
  {
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, HIGH);
    analogWrite(MOTOR2_PWM, motor2_pwm);
  }
  else if (MOTOR2_PWM < 0) // backward
  {
    digitalWrite(MOTOR2_ENA, HIGH);
    digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, -motor2_pwm);
  }
  else
  {
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, LOW);
    digitalWrite(MOTOR2_PWM, 0);
  }

}

void motor_control(int front_speed, int rear_speed)
{
  front_motor_control(front_speed);
  rear_motor_control(rear_speed);  
}


#include <SPI.h>
#define ENC1_ADD 22
#define ENC2_ADD 23

signed long encoder1count = 0;
signed long encoder1_error = 0;
signed long encoder1_error_d = 0;
signed long encoder1_target = 0;
signed long encoder1_error_old = 0; 
signed long encoder1_error_sum = 0; 

void initEncoders() {
  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT);
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC1_ADD,HIGH);
  digitalWrite(ENC2_ADD,HIGH);
  SPI.begin();
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC1_ADD,HIGH);       // Terminate SPI conversation 
  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC2_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC2_ADD,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder_no) 
{  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;   
  digitalWrite(ENC1_ADD + encoder_no-1,LOW);      // Begin SPI conversation
   // digitalWrite(ENC4_ADD,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  //digitalWrite(ENC4_ADD,HIGH);      // Begin SPI conversation
// Calculate encoder count
  count_value= ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8 ) + (long)count_4;
  return count_value;
}

void clearEncoderCount(int encoder_no) {    
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
}
/////////////////////////////////////// motor PID 제어  ////////////////////////////////////////////


// PID 게인 값은 조정 필요함
float Kp_motor = 6;
float Kd_motor = 4;
float Ki_motor = 2;

void front_motor_PID_control(void)
{
  encoder1count = readEncoder(1);
  encoder1_error = encoder1_target - encoder1count;
  encoder1_error_sum += encoder1_error;
  encoder1_error_d = encoder1_error - encoder1_error_old;
  encoder1_error_sum = (encoder1_error_sum >=  100) ?  100 : encoder1_error_sum;
  encoder1_error_sum = (encoder1_error_sum <= -100) ? -100 : encoder1_error_sum;
  front_motor_pwm = Kp_motor * encoder1_error + Kd_motor * encoder1_error_d + Ki_motor * encoder1_error_sum;
  front_motor_pwm = (front_motor_pwm >=  255) ?  255 : front_motor_pwm;
  front_motor_pwm = (front_motor_pwm <= -255) ? -255 : front_motor_pwm;
 if (fabs(encoder1_error) <= 2)
  {  
    encoder1_error_sum = 0;
  }
  else
  {
    front_motor_control(front_motor_pwm);    
  }
  encoder1_error_old = encoder1_error;   
}

///////////////////////////////////////  Steering PID 제어 /////////////////////////////////////////////

#define Steering_Sensor A15  // Analog input pin that the potentiometer is attached to
#define NEURAL_ANGLE 3
#define LEFT_STEER_ANGLE  -30
#define RIGHT_STEER_ANGLE  30
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10
#define AD_MIN ( 60 + 10)
#define AD_MAX (990 - 10)
#define alpha 0.3
#define max_val 990
#define min_val 60

// Low pass Filter variable
float old_avg = 0.0; // Xavg(k-1)
float avg     = 0.0; // Xvag(k)
int ad_value = 0;

float Kp = 0.97;
float Ki = 0.18;
float Kd = 2.18; //PID 상수 설정, 실험에 따라 정해야 함 중요!
double Setpoint, Input, Output; //PID 제어 변수
double error, error_old;
double error_s, error_d;
int pwm_output=0;

int sensorValue = 0;        // value read from the pot
int Steer_Angle_Measure = 0;        // value output to the PWM (analog out)
int Steering_Angle = 800;

void steer_motor_control(int motor_pwm)
{
  if( (sensorValue>= AD_MAX  ) || (sensorValue <= AD_MIN  ) )
  {
     digitalWrite(MOTOR3_ENA, LOW);
     digitalWrite(MOTOR3_ENB, LOW);
     analogWrite(MOTOR3_PWM, 0);
     return;    
  }
  if (motor_pwm > 0) // forward
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
    digitalWrite(MOTOR3_ENA, HIGH);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, -motor_pwm);
  }
  else // stop
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
  }
}

void Protect_steer(){
  if(max_val < avg) pwm_output = 0;
  else if(min_val > avg) pwm_output = 0;
  steer_motor_control(pwm_output);
}

void PID_Control() //steering
{
  error = Steering_Angle - avg ;
  error_s += error;
  error_d = error - error_old;
  error_s = (error_s >=  100) ?  100 : error_s;
  error_s = (error_s <= -100) ? -100 : error_s;
  pwm_output = Kp * error + Kd * error_d + Ki * error_s;
  pwm_output = (pwm_output >=  255) ?  255 : pwm_output;
  pwm_output = (pwm_output <= -255) ? -255 : pwm_output;

  if (fabs(error) <= 1.2)
  {
    steer_motor_control(0);
    error_s = 0;
  }
  else steer_motor_control(pwm_output);
  error_old = error;  
}

void steering_control()
{
  if (Steering_Angle <= LEFT_STEER_ANGLE - NEURAL_ANGLE)  Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  if (Steering_Angle >= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;
  PID_Control(); 
}


void control_callback()
{
  static boolean output = HIGH;
  digitalWrite(13, output);
  output = !output;
  front_motor_PID_control();

  ad_value = analogRead(A15); // Pin 지정 540
  // ad_value = 160;
  avg = alpha * old_avg + (1.0 - alpha)*ad_value;
  old_avg = avg;

  PID_Control();

  //Protect_steer();
 // ad_value = ad_value + 1;
}

void setup() {
    
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  // Front Motor Drive Pin Setup

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR1_ENB, OUTPUT);

  // Rear Motor Drive Pin Setup

  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR2_ENB, OUTPUT);

   //Steer
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR3_ENB, OUTPUT);  // L298 motor control PWM

  initEncoders();          // initialize encoder
  clearEncoderCount(1); 

  
  MsTimer2::set(100, control_callback); // 500ms period
  MsTimer2::start();
}

void loop(){
  // put your main code here, to run repeatedly:
 encoder1_target = 100;  // target encoder 값으로 조정 필요함
 Serial.print("Encoder 1 : ");    Serial.println(encoder1count);
 Serial.print("Encoder error 1 : ");    Serial.println(encoder1_error);
 Serial.print("pwm : ");    Serial.println(front_motor_pwm);
 Serial.print("sum : ");    Serial.println(encoder1_error_sum);

}