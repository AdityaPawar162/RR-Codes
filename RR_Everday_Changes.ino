//........//***********RRRRRRR**************LIBRARIES*****************//
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
#include "CytronMotorDriver.h"
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include<SoftwareSerial.h>
#include<SabertoothSimplified.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

RF24 radio(2, 4); // CE, CSN
const byte address[6] = "10101";

int rx;
int ry;
int lx;
int ly;
int up;
int down;
int right;
int left;
int green;
int blue;
int yellow;
int red = 0;
int r1;
int l1;
int l2;
int r2;
int r3;
int l3;
int start;
int back;
int xbox;
int sync;

struct record
{
  int rx;
  int ry;
  int lx;
  int ly;
  int up;
  int down;
  int right;
  int left;
  int green;
  int blue;
  int yellow;
  int red;
  int r1;
  int l1;
  int l2;
  int r2;
  int r3;
  int l3;
  int start1;
  int back1;
  int xbox1;
  int sync1;

};
record Record;

template<class T> inline Print& operator <<(Print &obj, T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

HardwareSerial& odrive_serial = Serial3 ;
ODriveArduino odrive(odrive_serial);

SoftwareSerial SWSerial(NOT_A_PIN, 9);
SabertoothSimplified ST(SWSerial);



// Configure the motor driver.
CytronMD motor1(PWM_DIR, 5, 32);
CytronMD motor2(PWM_DIR, 6, 34);
CytronMD motor3(PWM_DIR, 7, 36);
//************************PID GAINS******************//
//float Kp ;// 6;//13;;8
//float Ki ;//0.002;//0.003;//0.003
//float Kd ;//37;//30

//*********************DECLARATIONS************//
//int M1 , M2 , M3, m1, m2, m3;
int m1  , m2 , m3  , a , P , PX , PY;
unsigned long timer = 0;
unsigned long timer1 = 0;
unsigned long timer2 = 0;
float timeStep = 0.016498;
float yaw = 0;
int pre_error = 0;
int setpoint = 0 ;
int error ;
int Val , red_counter = 0 , red_flag = 0;
float pi = 3.14;
int bandW = 16 , gpad_flag = 0;
float ang = 0;
int var_100 = 100;
float rate, incre, PRO, DER, INT, output, pid ;
int Base_M = 0;
int rot_speed = 30 ;
int rotation_counter, rot_counter, rot_flag = 0;

int Pick_M = 0, up_speed = 35, down_speed = -35 ;

int atuate_flag = 0;

int load_flag = 0;
int load_count = 0;

int slow_flag = 0;
int slow_opposite_flag = 0;
int slow_side_flag = 0;
int slow_opposite_side_flag = 0;


int slow_speed_m1 = 0;
int slow_speed_m2 = 0;
int slow_speed_m3 = 0;
int prev = 0;
int m1_speed = 35;
int m2_speed = 40;
int m3_speed = 40;
//int yaw_zero_cnt = 0;


//______________________________________
float yawError ,   iterm , previousYaw ;
float kp = 6.5;
float ki = 0.00001 ;
float kd = 0.52;

int flag911 = 0;
int flag811 = 0;
int flag711 = 0;//For Ramp Mode
float flag611 = 0;//For R1
float flag511 = 0;//For L1
int flot_auto = 1; //BLDC
int flag11 = 0;//Gpad
int flag12 = 0;
int flag13 = 0;
int flag14 = 0;
int flag1 = 0; //locomotion angle 5
int flag2 = 0;
int flag3 = 0; //locomotion angle 90
int flag4 = 0;
float var0 , multiplier = 1 , rotate_motor = 0;
float var1 ;
int cnt = 0;//For BLDC
int cnt1 = 1;//For Vertical Angle DCV
int cnt2 = 0;//locomotion angle
int cnt3 = 0;//Temp variable
int cnt4 = 0;
int throw_flag = 0;
int yaw_flag = 0;
int k = 0;
//int red_flag = 0;
int blue_flag = 0;

//_______________________________________


void setup()
{
  Serial.begin(115200);
  SWSerial.begin(9600);
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(1);
  delay(200);
  pinMode(24, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(26, OUTPUT);

  //______________odrive_________________

  odrive_serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");
  odrive.SetVelocity(0, 0);
  odrive.SetVelocity(1, 0);


  //________________NRF_____________________
  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setAutoAck(false);
  radio.setDataRate( RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();              //This sets the module as receiver
  Serial.print("SET UP START");
  delay(1000);

  //________________________________________
}

void loop()
{
  if (radio.available()) {
    radio.read(&Record , sizeof(Record));
    Serial.print("    Inside Radio Available  ");
    k = 1;
  }
  else {
    k = 0;
  }

  ST.motor(1, Pick_M);
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;

  rx = Record.rx ;
  ry = Record.ry ;
  lx = Record.lx*0.40 ;
  ly = Record.ly*0.40;
  r1 = Record.r1;
  l1 = Record.l1;
  yellow = Record.yellow ;
  red = Record.red;
  green = Record.green;
  blue = Record.blue;
  r2 = Record.r2 ;
  l2 = Record.l2 ;
  up = Record.up;
  down = Record.down;
  right = Record.right;
  left = Record.left;
  r3 = Record.r3;
  l3 = Record.l3;
  start = Record.start1;
  back = Record.back1;
  xbox = Record.xbox1;
  sync = Record.sync1;
  ry = -ry;
  ly = - ly;

  //*****************LOCOMOTION******************


  Serial.print(" Red = ");
  Serial.print(red);

  if (r1 == 1 && flot_auto == 0) {
    var0 = 17;
    odrive.SetVelocity(0, var0);
    odrive.SetVelocity(1, -var0);
    cnt++;
    flot_auto = 1;
  }
  if (r1 == 0 && flot_auto == 1)flot_auto = 0;


  if (cnt % 2 != 0) {
    Serial.print("IN BLDC MODE");
    //    __________________________________________BLDC ADJUSTMENTS _____________________________________________
    if (down == 1 && flag11 == 0)
    {
      var0 -= 0.1;
      odrive.SetVelocity(0, var0);
      odrive.SetVelocity(1, -var0);
      flag11 = 1;
      //      Serial.print(" Increment of 0axis");
    }
    if (up == 1 && flag12 == 0)
    {
      var0 += 0.1;
      odrive.SetVelocity(0, var0);
      odrive.SetVelocity(1, -var0);
      flag12 = 1;
      //      Serial.print(" Decrement of 0axis");
    }
    if (right == 1 && flag13 == 0)
    {
      var0 += 1;
      odrive.SetVelocity(0, var0);
      odrive.SetVelocity(1, -var0);
      flag13 = 1;
      //      Serial.print(" Increment of 1axis");
    }
    if (left == 1 && flag14 == 0)
    {
      var0 -= 1;
      odrive.SetVelocity(0, var0);
      odrive.SetVelocity(1, -var0);
      flag14 = 1;
      //      Serial.print(" Decrement of 1axis");
    }
    if (down == 0 && flag11 == 1)   flag11 = 0;
    if (up == 0 && flag12 == 1)   flag12 = 0;
    if (left == 0 && flag14 == 1)   flag14 = 0;
    if (right == 0 && flag13 == 1)   flag13 = 0;

    //    _______________________________________________GRAB_____________________________________________
    if (l1 == 1 && load_flag == 0)
    {
      load_count++;
      load_flag = 1;
    }
    if (l1 == 0 && load_flag == 1) {
      load_flag = 0;
    }
    //    _________________________________________________THROW____________________________________________________
    if (blue == 1 && throw_flag == 0) {
      digitalWrite(26, HIGH);
      throw_flag = 1;
      Serial.print(" THROW ");
    }
    if (blue == 0 && throw_flag == 1) {
      digitalWrite(26, LOW);
      throw_flag = 0;
    }
  //   ________________________________________________ANGLE CYLINDER ______________________________________
    if (red == 1 && flag511 == 0)//Use flag511 for red
    {
      cnt1++;
      flag511 = 1;
    }
    if (red == 0 && flag511 == 1)    flag511 = 0;
    if (cnt1 % 2 == 0)  {
      Serial.print("VERTICAL");
      digitalWrite(24, HIGH);
    }
    if (cnt1 % 2 == 1)  digitalWrite(24, LOW);

    //    _________________________________________________ BLDC MODE OVER __________________________________________

  }
  Serial.print("Var0 = ");
  Serial.print(var0);

  if (cnt % 2 == 0) {
    Serial.print("~BLDC MODE");
    var0 = 0;
    odrive.SetVelocity(0, var0);
    odrive.SetVelocity(1, -var0);
    // __________________________________________________ Slow Locomotion _________________________________________________
    if (up == 1 && slow_flag == 0) {
      slow_speed_m2 = -m2_speed;
      slow_speed_m3 = -m3_speed;
      slow_flag = 1;
      multiplier = 1;
      //      Serial.print("SlowF");
    }

    if (down == 1 && slow_opposite_flag == 0) {
      slow_speed_m2 = m2_speed;
      slow_speed_m3 = m3_speed;
      slow_opposite_flag = 1;
      //      Serial.print("SlowD");
    }

    if (right == 1 && slow_side_flag == 0) {
      slow_speed_m1 = m1_speed;
      slow_speed_m2 = m2_speed;
      slow_speed_m3 = -m3_speed;
      slow_side_flag = 1;
      //      Serial.print("SlowR");
    }

    if (left == 1 && slow_opposite_side_flag == 0) {
      slow_speed_m1 = -m1_speed;
      slow_speed_m2 = -m2_speed;
      slow_speed_m3 = m3_speed;
      slow_opposite_side_flag = 1;
      //      Serial.print("SlowL");
    }

    if ((up == 0 && slow_flag == 1) || (down == 0 &&  slow_opposite_flag == 1 ) || ( right == 0 && slow_side_flag == 1 ) || (left == 0 && slow_opposite_side_flag == 1))
    {
      slow_speed_m1 = 0;
      slow_speed_m2 = 0;
      slow_speed_m3 = 0;
      slow_flag = 0;
      slow_opposite_flag = 0;
      slow_side_flag = 0;
      slow_opposite_side_flag = 0;

    }

        //    _______________________________________________GRAB_____________________________________________
    if (l1 == 1 && load_flag == 0)
    {
      load_count++;
      load_flag = 1;
    }
    if (l1 == 0 && load_flag == 1) {
      load_flag = 0;
    }
    
  }

  //   _______________________________________________ Picking Logic______________________________________
  if (yellow == 1 || green == 1) {
    if (yellow == 1) {
      Pick_M = 35;
    }
    if (green == 1) {
      Pick_M = -30;
    }
    else {
      Pick_M = 0;
    }
  }


  //_________________________________________________ GRAB REST ________________________________________
  if (load_count % 2 != 0) {
    digitalWrite(22, HIGH);
    Serial.print("GRAB");
  }
  if (load_count % 2 == 0) {
    digitalWrite(22, LOW);
  }

  //______________________________ Rotate _____________________________________

  if ( l2 == 1  ) {
    rotate_motor = -30;
    setpoint = yaw;
    multiplier = 0;
  }
  else if ( r2 == 1 ) {
    rotate_motor = 30;
    setpoint = yaw;
    multiplier = 0;
  }
  else if ( l2 == 0 && r2 == 0 ) {
    rotate_motor = 0;
    multiplier = 1;
  }



  //_________________________________90 Degree__________________________________
  if (red == 1 && red_flag == 0 && cnt % 2 == 0) {
    multiplier = 1;
    setpoint += 90;
    red_flag = 1;
  }
  red_flag = red;

  if (blue == 1 && blue_flag == 0 && cnt % 2 == 0) {
    multiplier = 1;
    setpoint -= 90;
    blue_flag = 1;
  }
  blue_flag = blue;


  ////////////// PID  ///////////////////
  yawError = setpoint - yaw;
  iterm += yawError;
  if ( yawError > -10 and yawError < 10) {
    kp = 28.5;
    ki = 0.00001;
    kd = 22.8;
  }
  else {
    kp = 5.7;
    ki = 0.00001;
    kd = 16;    //    kp = 2.0;
    //    ki = 0.0;
    //    kd = 1.5;
  }
  pid = kp * yawError +  ki * iterm + kd * (previousYaw - yaw);
  previousYaw = yaw;
  pid = map(pid , -550 , 550 , -150 , 150 );
  pid = constrain ( pid , -150 , 150 );


  ////////////////*********EQUATIONS***********////////////////////
  a = (lx+rx)*(lx+rx) + (ly+ry) * (ly+ry) ;
  P  = sqrt(a) ;
  P = constrain(P, 0, 200);
  float l = atan2(ly+ry, lx+rx);
  int ang = l * 180 / 3.14;//IN Degree
  ang += yaw;
  if (((rx < 0 && ry < 0)||(rx > 0 && ry < 0)) || ((lx < 0 && ly < 0) || (lx > 0 && ly < 0)) ){
    ang = ang + 360;
  }

  PX = P * cos(ang * 3.14 / 180);
  PY = P * sin(ang * 3.14 / 180);
  m1 = 2 * PX / 3    ;
  m2 =  -PX / 3 + 0.5776 * PY ;
  m3 = -PX / 3 - 0.57736 * PY ;

  m1 = map(m1 , 90 , -90 , 255 , -255 );
  m2 = map(m2 , 90 , -90 , 255 , -255 );
  m3 = map(m3 , 90 , -90 , 255 , -255 );
  m1 = m1 - pid;
  m2 = m2 - pid;
  m3 = m3 - pid;
  //  m1 = constrain(m1 , 30 , -30);
  //  m2 = constrain(m2 , 30 , -30);
  //  m3 = constrain(m3 , 30 , -30);


  int M1 = -m1 * multiplier - rotate_motor + slow_speed_m1;
  int M2 = m2 * multiplier + rotate_motor + slow_speed_m2;
  int M3 = -m3 * multiplier - rotate_motor + slow_speed_m3;

  //  Serial.print(" Rx = ");
  //  Serial.print(rx);
  //  Serial.print("   Ry = ");
  //  Serial.print(ry);
  Serial.print(" left= ");
  Serial.print(left);
  Serial.print("   right = ");
  Serial.print(right);
  Serial.print(" l1 = ");
  Serial.print(l1);
  Serial.print(" r1 = ");
  Serial.print(r1);
  //  Serial.print("k");
  //  Serial.print(k);
  Serial.print("  M1 ");
  Serial.print(M1);
  Serial.print(" ");
  Serial.print(M1);
  Serial.print("  M2 ");
  Serial.print(M2);
  Serial.print("  M3 ");
  Serial.println(M3);
  Serial.print("Setpoint = ");
  Serial.print(setpoint);
  Serial.print("Yaw ");
  Serial.print(yaw);
  Serial.print(" PID = ");
  Serial.print(pid);
  Serial.print("KP ");
  Serial.print(kp);
  Serial.print("Kd");
  Serial.print(kd);
  Serial.print("Ki");
  Serial.print(ki);
  Serial.print("multiplier");
  Serial.print(multiplier);




  motor1.setSpeed(M1);
  motor2.setSpeed(M2);
  motor3.setSpeed(M3);




  //  Serial.println();

}
