#include <MsTimer2.h>

const int BUFFER_SIZE = 4;

// Potentiometer ADC value  
#define alpha 0.3
#define max_val (989-10)
#define min_val (100)

// Steer DC motor pin
#define MOTOR3_PWM 8 
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

// Low pass Filter variable
float old_avg = 0.0; // Xavg(k-1)
float avg     = 0.0; // Xvag(k)
int ad_value = 0;

// PID variable
float Kp = 1.0;
float Ki = 0.1;
float Kd = 1.0; //PID 상수 설정, 실험에 따라정해야 함 
double error, error_old;
double error_s, error_d;

int Steering_Angle = 500; // Front Moter goal angle

int pwm_output = 0; //  PWM value

void Timer_ISR(void){
  ad_value = analogRead(A15); // Pin 지정 540
 // ad_value = 160;
  avg = alpha * old_avg + (1.0 - alpha)*ad_value;
  old_avg = avg;

  PID_Control();

  Protect_steer();
 // ad_value = ad_value + 1;
}

void Protect_steer(){
  if(max_val < avg) pwm_output = 0;
  else if(min_val > avg) pwm_output = 0;
  steer_motor_control(pwm_output);
}

void PID_Control()
{
  error = Steering_Angle - avg ;
  error_s += error;
  error_d = error - error_old;
  error_s = (error_s >=  100) ?  100 : error_s;
  error_s = (error_s <= -100) ? -100 : error_s;
  pwm_output = Kp * error + Kd * error_d + Ki * error_s;
  pwm_output = (pwm_output >=  250) ?  250 : pwm_output;
  pwm_output = (pwm_output <= -250) ? -250 : pwm_output;

  if (fabs(error) <= 3)
  {
    steer_motor_control(0);
    error_s = 0;
  }
  else steer_motor_control(pwm_output);
  error_old = error;  
}

void steer_motor_control(int motor_pwm)
{
  if( (avg>= max_val  ) || (avg <= min_val  ) )
  {
    // Serial.println("HH");
     digitalWrite(MOTOR3_ENA, LOW);
     digitalWrite(MOTOR3_ENB, LOW);
     analogWrite(MOTOR3_PWM, 0);
     return;    
  }

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

void setup() {
  Serial.begin(115200);

  MsTimer2::set(10,Timer_ISR);
  MsTimer2::start();
   //Steer
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR3_ENB, OUTPUT);  // L298 motor control PWM
}

void loop() {
  
   if (Serial.available() > 0) {
    char buf[BUFFER_SIZE]={0};
    // read the incoming bytes:
    Serial.readBytes(buf, BUFFER_SIZE);
    Steering_Angle=atoi(buf);
    Serial.println(Steering_Angle);
  }
  //pwm_output = Kp * error + Kd * error_d + Ki * error_s;
  Serial.print(avg);Serial.print(" ");
  Serial.print(error);Serial.print(" ");
  Serial.print(error_s);Serial.print(" ");
  Serial.print(error_d);Serial.print(" ");
  Serial.println(pwm_output);
  
  
}