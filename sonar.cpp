#include <NewPing.h>

#define SONAR_NUM 3        // Number of sensors.
#define MAX_DISTANCE 100    // Maximum distance (in cm) to ping.
#define FILTER_SIZE 30     // Size of the filter for averaging distances.

NewPing sonar[SONAR_NUM] = {
  NewPing(54,57, MAX_DISTANCE),
  NewPing(55,58, MAX_DISTANCE),
  NewPing(56,59, MAX_DISTANCE)// triger echo
};

float distanceValues[SONAR_NUM][FILTER_SIZE] = {0};
int currentIndex = 0;
const int STOP_THRESHOLD = 100; // Threshold distance for stopping.
unsigned long distance[3] = {0};
unsigned long avg_sonar[3] = {0};
int flag_sonar_stop=0;

void setup() {
  Serial.begin(9600);
  TCCR3B = TCCR3B & B11111000 | B00000001;   // for PWM frequency of 31372.55 Hz, front motor
  TCCR4B = TCCR4B & B11111000 | B00000001;   // for PWM frequency of 31372.55 Hz  steering motor
}

void Sonar_stop(void)
{
    distance[0] = sonar[0].ping_cm();
  distance[1] = sonar[1].ping_cm();
  distance[2] = sonar[2].ping_cm();

  distanceValues[0][currentIndex] = distance[0];
  distanceValues[1][currentIndex] = distance[1];
  distanceValues[2][currentIndex] = distance[2];

 
  Serial.print("sona1:");
  Serial.print(distance[0]);
  Serial.print("  ");
  Serial.print("sona2_:");
  Serial.print(distance[1]);
  Serial.print("  ");
  Serial.print("sona3_:");
  Serial.println(distance[2]);
 

  currentIndex = (currentIndex + 1) % FILTER_SIZE;

  if (currentIndex == 0) {
    float sum[3] = {0};
    for (int i = 0; i < 3; i++) {
      for (int cnt = 0; cnt < FILTER_SIZE; cnt++) {
        sum[i] += distanceValues[i][cnt]; // Fixed calculation here.
      }
      avg_sonar[i] = sum[i] / FILTER_SIZE;
    }


    if ( 0< avg_sonar[0] && avg_sonar[0] <= STOP_THRESHOLD || 0 < avg_sonar[1] && avg_sonar[1] <= STOP_THRESHOLD || 0 < avg_sonar[2] && avg_sonar[2] <= STOP_THRESHOLD) { // Fixed condition here.
    
    flag_sonar_stop=1;

   } else {
      flag_sonar_stop=0;
    }
  }
  

}

void loop() {

  Sonar_stop();
  Serial.println(flag_sonar_stop);
}