///////////////////////////////////////  Sonar Sensor /////////////////////////////////////////////

#include <NewPing.h>

#define SONAR_NUM 3        // Number of sensors.
#define MAX_DISTANCE 100    // Maximum distance (in cm) to ping.
#define FILTER_SIZE 10     // Size of the filter for averaging distances.

NewPing sonar[SONAR_NUM] = {
  NewPing(54,57, MAX_DISTANCE),
  NewPing(55,58, MAX_DISTANCE),
  NewPing(56,59, MAX_DISTANCE)// triger echo
};


const int STOP_THRESHOLD = 60; // Threshold distance for stopping.
unsigned long distance[3] = {0};
int flag_sonar_stop=0;


void Sonar_stop(void)
{
  distance[0] = sonar[0].ping_cm();
  distance[1] = sonar[1].ping_cm();
  distance[2] = sonar[2].ping_cm();

/*
  Serial.print("sona1:");
  Serial.print(distance[0]);
  Serial.print("  ");
  Serial.print("sona2_:");
  Serial.print(distance[1]);
  Serial.print("  ");
  Serial.print("sona3_:");
  Serial.println(distance[2]);
 */


    if ( 7< distance[0] && distance[0] <= STOP_THRESHOLD || 7< distance[1] && distance[1] <= STOP_THRESHOLD || 7< distance[2] && distance[2] <= STOP_THRESHOLD) { // Fixed condition here.
    
    flag_sonar_stop=1;

   } else {
      flag_sonar_stop=0;
    }
  
  


  

}

















///////////////////////////////////////  Motor Fuction /////////////////////////////////////////////

#define MOTOR1_PWM 5 //moter 1 front
#define MOTOR1_ENA 6
#define MOTOR1_ENB 7

#define MOTOR2_PWM 2 //moter 2 back
#define MOTOR2_ENA 3
#define MOTOR2_ENB 4

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







///////////////////////////////////////  Encoder SPI Setup /////////////////////////////////////////////

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

long readEncoder(int encoder_no) // forward(-), backward(+)
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











///////////////////////////////////////  Front Motor PID Control /////////////////////////////////////////////

// PID 게인 값은 조정 필요함
float Kp_motor = 3.0;
float Kd_motor = 0.1;
float Ki_motor = 0.1;

void front_motor_PID_control(void)
{
  encoder1count = -readEncoder(1); // encoder forward(++) back(--)
  encoder1_error = encoder1_target - encoder1count;
  encoder1_error_sum += encoder1_error;
  encoder1_error_d = encoder1_error - encoder1_error_old;
  encoder1_error_sum = (encoder1_error_sum >=  130) ?  130 : encoder1_error_sum;
  encoder1_error_sum = (encoder1_error_sum <= -130) ? -130 : encoder1_error_sum;
  front_motor_pwm = Kp_motor * encoder1_error + Kd_motor * encoder1_error_d + Ki_motor * encoder1_error_sum;
  front_motor_pwm = (front_motor_pwm >=  250) ?  250 : front_motor_pwm;
  front_motor_pwm = (front_motor_pwm <= -250) ? -250 : front_motor_pwm;
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











///////////////////////////////////////  Steering PID Control /////////////////////////////////////////////

#define Steering_Sensor A15  // Analog input pin that the potentiometer is attached to
#define NEURAL_ANGLE 3
#define LEFT_STEER_ANGLE   30
#define RIGHT_STEER_ANGLE  -30
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10
#define AD_MIN (80 + 10)
#define AD_MAX (989 + 10)
#define alpha 0.3

// Low pass Filter variable
float old_avg = 0.0; // Xavg(k-1)
float avg     = 0.0; // Xvag(k)
int ad_value = 0;


// PID variable
float Kp = 2.5;
float Ki = 0.4;
float Kd = 0.6; 
double error=0, error_old=0;
double error_s=0, error_d=0;
int pwm_output=0;
int Steering_Angle = 500; // Front Moter goal angle

void steer_motor_control(int motor_pwm)
{
  /*
   if( (avg>= AD_MAX  ) || (avg <= AD_MIN  ) )
  {
     digitalWrite(MOTOR3_ENA, LOW);
     digitalWrite(MOTOR3_ENB, LOW);
     analogWrite(MOTOR3_PWM, 0);
     return;    
  }
  */

  if (motor_pwm > 0) // Right
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // Left
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

void PID_Control() //steering
{
  error = Steering_Angle - avg ;
  error_s += error;



  error_d = error - error_old;
  error_s = (error_s >=  100) ?  100 : error_s;


  error_s = (error_s <= -100) ? -100 : error_s;
  pwm_output = Kp * error + Kd * error_d + Ki * error_s;
  pwm_output = (pwm_output >=  250) ?  250 : pwm_output;
  pwm_output = (pwm_output <= -250) ? -250 : pwm_output;

  if (fabs(error) <= 3) //0.2?
  {
    steer_motor_control(0);
    error_s = 0;
  }
  else steer_motor_control(pwm_output);
  error_old = error;  
}





///////////////////////////////////////  serial /////////////////////////////////////////////

//Serial data
int cnt=-1;
unsigned char buf[30];

union
{
  float data ;
  char  bytedata[4];
    
} m_car_speed_float;

union
{
    short data ;
    char  bytedata[2];
    
} m_car_angle_int16;

















///////////////////////////////////////  callback /////////////////////////////////////////////
#include <MsTimer2.h>


float velocity=0;     //ros로 받아온 velocity -> 0.1 단위
short steer_angle=0;  // ros로 받아온 각도 -> 2 단위 , 좌측(30), 우측(-30)


void control_callback()
{
  

  // front motor contral
  //encoder1_target += velocity * 97.5; 
  front_motor_PID_control();
  
  //motor_control(130,0); // least motor pwm
  if(flag_sonar_stop==1)
  {
    Serial.println("stop");
  }
  else if(flag_sonar_stop==0)
  {
    encoder1_target += 9.75; //0.1m/s19.5
    //Serial.println(flag_sonar_stop); 
  }
  
  if(Serial1.available()){
    for(int i=0; i<9; i++){
      buf[i]= Serial1.read();
      //Serial.println(buf[i]);
      }
    }
   if((buf[0]=='#') &&(buf[1]=='C') && ( buf[4] == '*') )
    {
      m_car_angle_int16.bytedata[0] = buf[2];
      m_car_angle_int16.bytedata[1] = buf[3];
    
      
      Serial.print(m_car_angle_int16.data);  Serial.println(" ");  
    } 

    steer_angle = m_car_angle_int16.data;

    if(steer_angle>=15) steer_angle=15;
    if(steer_angle<=-17) steer_angle=-17;

  // steering control
  ad_value = analogRead(A15); 
  avg = alpha * old_avg + (1.0 - alpha)*ad_value;
  old_avg = avg;


  Steering_Angle = map(steer_angle, LEFT_STEER_ANGLE,RIGHT_STEER_ANGLE, 100, 900); //측정값 0도 == 500


  PID_Control();

}













///////////////////////////////////////  setup and loop /////////////////////////////////////////////


void setup() {

  TCCR3B = TCCR3B & B11111000 | B00000001;   // for PWM frequency of 31372.55 Hz, front motor
  TCCR4B = TCCR4B & B11111000 | B00000001;   // for PWM frequency of 31372.55 Hz  steering motor
  Serial.begin(115200);
  Serial.print("Serial Port Connected!");
  Serial1.begin(115200);
  //Serial1.print("Serial1 Port Connected!");
  // put your setup code here, to run once:
  
  // Front Motor Drive Pin Setup

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR1_ENB, OUTPUT);

  // Rear Motor Drive Pin Setup

  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR2_ENB, OUTPUT);

  // Steer Motor Drive Pin Setup
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR3_ENB, OUTPUT);  // L298 motor control PWM

  // Encoder initialize
  initEncoders();          // initialize encoder
  clearEncoderCount(1); 

  
  MsTimer2::set(50, control_callback); // 100ms period
  MsTimer2::start();
}

const int BUFFER_SIZE = 4;

void loop(){
  /*
    if (Serial.available() > 0) {
    char buf[BUFFER_SIZE]={0};
    // read the incoming bytes:
    Serial.readBytes(buf, BUFFER_SIZE);
    Steering_Angle=atoi(buf);
    Serial.println(Steering_Angle);
  }*/
  //Serial_read_data();
  //Sonar_stop();  
   Sonar_stop();
  //Serial.println(velocity);
  //Serial1.println(steer_angle);
  //Serial.println(Steering_Angle);
  

  //Serial.println(encoder1count);
  /*
  Serial.print(encoder1_error);Serial.print(" ");
  Serial.print(encoder1_error_sum);Serial.print(" ");
  Serial.print(encoder1_error_d);Serial.print(" ");
  Serial.print(front_motor_pwm);Serial.print(" ");
  Serial.println(encoder1_target);
  */

}

