#include <MsTimer2.h>
#define alpha 0.85
#define AD_MIN (60 + 10)
#define AD_MAX (990 - 10)

float avg_old = 0.0; // xavg(k-1)
float avg     = 0.0; // xavg(k)
int ad_value  = 0;

void setup() {
  Serial.begin(750);
  MsTimer2::set(10,Timer_ISR); // 10ms 
  MsTimer2::start();
}
void Timer_ISR(void)
{
  avg = alpha * avg_old + (1.0 - alpha)*ad_value;
  avg_old = avg;  
}
void loop() {

  ad_value = analogRead(A15);
  Serial.print("Ad_vale = "); 
  Serial.print(','); Serial.println(avg);
}
