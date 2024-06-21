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



void setup() {
// put your setup code here, to run once:
//initialize both serial ports
Serial.begin(115200);
Serial1.begin(115200);
}

void loop() {
// put your main code here, to run repeatedly:
//read from port 1, send to port 0

/*f (Serial1.available()){
char inByte = Serial1.read();
Serial.write(inByte);
}*/

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
  delay(50);


}