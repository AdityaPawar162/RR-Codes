//Aditya Pawar FE Co RRL, [15-11-2022 13:33]
#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include <SPI.h>
#include "hidjoystickrptparser.h"
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
#include <Wire.h>
#include <MPU6050.h>
//################################################################################ JOYSTICK ###################################################################################
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);
//################################################################################ JOYSTICK ###################################################################################

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ SABERTOOTH $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
SoftwareSerial SWSerial(NOT_A_PIN, 5); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSethre=ial as the serial port.
SoftwareSerial SW1Serial(NOT_A_PIN, 4); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST1(SW1Serial); // Use SWSerial as the serial port.
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ SABERTOOTH $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ENCODER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Encoder A
int encoderpinxA = 2;
int encoderpinxB = 3, start_flag = 0;
volatile long currentPositionx = 0;
////Rncoder B
int encoderpinyA = 18;
int encoderpinyB = 19;
volatile long currentPositiony = 0;
int out, inp, output, input, data, setPoint;
double sum_error;
float error_r  , derivative_r  , integral_r   , previous_error_r ;

//float error  , derivative  , integral   , previous_error ;
//float error_r  , derivative_r  , integral_r   , previous_error_r ;
//int encoderpinxA = 2;
//int encoderpinxB = 3;
//volatile long currentPositionx = 0;
//
//int encoderpinyA = 18;
//int encoderpinyB = 19;
//volatile long currentPositiony = 0;


float kp  ;  //0.89;
float ki ;
float kd ; //  5 ;
signed long out_x ;
signed long out_y ;
int flag_auto = 0 ;
int button_flag;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ENCODER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  IMU !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// PID GAIN DECLERATION ********************
float error  , derivative  , integral   , previous_error ;
float Kp ; // =   8.5;        // 5&& 3.4
float Ki  ;//  = 0.5;           //0.00001;     // 0.00049 ,0.008     && 0.0042  ,0.0000035
float Kd   ; //=   38;        // 0.698, 0.010  &&   0.78
// PID GAIN DECLERATION ********************
float setpoint = 0 ;
float pi = 3.14 ;
// MPU DECLARATION **************************
MPU6050 mpu;
// Timers
unsigned long timer = 0;
float timeStep = 0.02481;
float yaw = 0;


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  IMU !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

float motor1 , motor2 , motor3 , motor4 ;
int Max_speed = 25 ;
int right_thumb_flag = 0;
int left_thumb_flag = 0;
int head_counter = 0;
int pid_flag = 1 ;
int counter_mode = 0 ;
int flot_auto = 1 ;
int counter, flag_button;
int arrx[3], arry[3];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

#if !defined(MIPSEL)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif

  Serial.println("Start");
  while (Usb.Init() == -1)
    Serial.println("OSC did not start.");
  delay(200);
  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);
  SWSerial.begin(9600);
  SW1Serial.begin(9600);


  // GYRO setup*************************
  Serial.begin(115200);
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  // GYRO SETUP *******************************

  pinMode(encoderpinxA, INPUT_PULLUP);
  pinMode(encoderpinxB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderpinxA), X , RISING);

  pinMode(encoderpinyA, INPUT_PULLUP);
  pinMode(encoderpinyB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderpinyA), Y, RISING);

}

void loop() {
  // put your main code here, to run repeatedly:
  Vector norm = mpu.readNormalizeGyro();
  yaw = yaw + norm.ZAxis * timeStep;
  //################################################################################ JOYSTICK ###################################################################################
  Usb.Task();
    float Buttons = JoyEvents.bu;
  float blue = Joy.blue;
  float lxa =  JoyEvents.lx;
  int yellow = Joy.yellow;  //  map( JoyEvents.lx, 0, 127,0, 1023);      //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f);
  int   LXjoy = map(lxa , 0 , 255 , -50 , 50 );
  //  Serial.print("\tLeft Joystick X: ");
  if (LXjoy > -10 && LXjoy < 10 ) {
    LXjoy  = 0 ;
  }
  //  Serial.print(LXjoy);
  float lya = JoyEvents.ly;           //map(JoyEvents.ly, 0, 127, 0, 1023);                   // map(JoyEvents.Z1, 0, 0xFF, 0.f, 255.f);
  int LYjoy = map(lya , 0 , 255 , Max_speed , -Max_speed );
  //  Serial.print("\tLeft Joystick Y: ");
  if (LYjoy > -10 &&  LYjoy < 10 ) {
    LYjoy  = 0 ;
  }
  //  Serial.print(LYjoy);
  float rxa = JoyEvents.rx;          //         map(JoyEvents.rx, 0, 127, 0, 1023); // map(JoyEvents.Z2, 0, 0xFF, 0.f, 255.f);
  int RXjoy = map(rxa , 0 , 255 , Max_speed , -Max_speed );
  //  Serial.print("\tRight Joystick X: ");
  if (RXjoy > -10 && RXjoy < 10 ) {
    RXjoy  = 0 ;
  }
  //  Serial.print(RXjoy );
  float rya = JoyEvents.ry;          // map(JoyEvents.ry, 0, 127, 0, 1023); // map(JoyEvents.Rz, 0, 0xFF, 0.f, 255.f);
  int RYjoy = map(rya , 0 , 255 , -Max_speed ,  Max_speed );
  //  Serial.print("\tRight Joystick Y: ");
  if (RYjoy > -10 && RYjoy < 10 ) {
    RYjoy  = 0 ;
  }
  //  Serial.print(RYjoy);
  int right_thumb = Joy.rt;
  int left_thumb = Joy.lt;
  //  float L2 = Joy.lt;
  //  float R2 = Joy.rt;

  //****************  right_thumb BUTTON CONDITION****************************

  //  Serial.print("\t Head Counter = ");
  //  Serial.print(head_counter);

  if (right_thumb == 0 && right_thumb_flag == 1 )
  {
    head_counter++;
    right_thumb_flag = 0;
  }
  if (right_thumb  == 1 && right_thumb_flag == 0 )
  {
    right_thumb_flag = 1;
  }

  if (left_thumb == 0 && left_thumb_flag == 1 )
  {
    head_counter--;
    left_thumb_flag = 0;
  }
  if (left_thumb  == 1 && left_thumb_flag == 0 )
  {
    left_thumb_flag = 1;
  }


  if (right_thumb == 0 && right_thumb_flag == 1 )
  {
    head_counter++;
    right_thumb_flag = 0;
  }
  if (right_thumb  == 1 && right_thumb_flag == 0 )
  {
    right_thumb_flag = 1;
  }

  if ( yellow == 1 && flot_auto == 0)
  {
    counter_mode ++;
    flot_auto = 1 ;

  }
  if ( yellow == 0 && flot_auto == 1 )
  {

    flot_auto = 0 ;

  }
  //  if (counter % 2 == 0 )
  //  {
  //    setpoint = 0;
  //    pid_flag = 0;
  //  }
  //  if (counter % 2 == 1 )
  //  {


  //################################################################################ JOYSTICK ###################################################################################

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  IMU !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  setpoint = 45;
  //    pid_flag = 0 ;
  //  }
  setpoint = setpoint * head_counter ;
  int YO =  yaw;
  if ( setpoint == YO  )
  {
    Kp =   8.5;        // 5&& 3.4
    Ki = 0.5;           //0.00001;     // 0.00049 ,0.008     && 0.0042  ,0.0000035
    Kd =   38;        // 0.698, 0.010  &&   0.78
    Serial.print(" locomotion is on ");
    pid_flag = 1 ;
  }
  else {
    Kp =   4.95;         //8.5;  // 5&& 3.4
    Ki = 0.27;           //0.00001;     // 0.00049 ,0.008     && 0.0042  ,0.0000035
    Kd =  34.69;
    Serial.print(" lets change the head");
  }
  //PID ON MPU 6050 start ***********
  //  Serial.print("  Yaw = ");
  //  Serial.print(yaw);

  error =  setpoint + yaw ;
  if (error < 35 && error > -35 ) {
    integral = integral +  error ;
  }
  integral = constrain(integral, -20, 20);
  //  Serial.print("integral value = ");
  //  Serial.print( integral );

  derivative = error -  previous_error ;
  //  Serial.print("derivative  = ");
  //  Serial.print(derivative);

  previous_error = error ;

  //  Serial.print("\t right_thumb=");
  //  Serial.print(right_thumb);
  //
  //  Serial.print(" setpoint  =");
  //  Serial.print(setpoint);
  float  pid = Kp * error +  Kd * derivative  +   Ki * integral ;
  pid = constrain(pid, -60, 60);
  pid = -pid;
  //Serial.print("\t error = ");
  //  Serial.print(error);
  //  Serial.print("\t pid = ");
  //  Serial.print(pid);

  //PID ON MPU 6050 end *************************************************************************************************************

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  IMU !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ VECTOR ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // polar logic begin *********************************
  float J_value = sqrt(sq(RYjoy) + sq(RXjoy));
  J_value = abs(J_value);
  //  Serial.print("\t red_butten = ");
  //  Serial.print(yellow);
  Serial.print("\tcurrentPositiony : ");
  Serial.print( currentPositiony  );
  Serial.print(" \tcurrentPositionx : ");
  Serial.print(currentPositionx );
  float start = Joy.st;
  if (start == 1 && start_flag == 0)
  {
    currentPositionx = 0;
    currentPositiony = 0;
    start_flag = 1;
  }
  if (start == 0 && start_flag == 1)
  {
    start_flag = 0;
  }



  float    An, speedt ;
  float currentPosition_R = sqrt(sq( out_y - currentPositiony ) + sq( out_x - currentPositionx ));
  //  Serial.print("\t currentPosition_R ");
  //  Serial.print(currentPosition_R);
  //
  //  Serial.print("\t counter_mode: ");



 

  //  Serial.print("Shit mode");
  //  Serial.print(counter_mode);
  //
  //  Serial.print( "\t flag_auto");
  //  Serial.print( flag_auto);

  if (counter_mode % 2 == 0 )
  {
   if (blue == 1 && button_flag == 0)
  {
    counter++;
    arrx[counter - 1] = currentPositionx;
    arry[counter - 1] = currentPositiony;
    Serial.print("counter  ");
    Serial.print(counter);
    Serial.print("position recorded");
    Serial.print("arr_in_x");
    Serial.print(arrx[counter - 1]);
    Serial.print("arr_in_y");
    Serial.print(arry[counter - 1]);

    button_flag = 1;
  }

  if (blue == 0 && button_flag == 1)
  {
    button_flag = 0;
  }
    An =   atan2 (  RYjoy ,  RXjoy );  ;
    speedt = Max_speed * ( J_value / ( 5  ) );
    flag_auto = 0;
    Serial.print("\t MANUAL MODE ON");
  }
  //*****************************************************************




  if (counter_mode % 2 == 1  )
  {
    float currentPosition_R = sqrt(sq( out_y - currentPositiony ) + sq( out_x - currentPositionx ));
    if (blue == 1 && button_flag == 0)
  {
    counter--  ;
   out_x= arrx[counter - 1] ;
    out_y = arry[counter - 1] ;
    Serial.print("counter  ");
    Serial.print(counter);
    Serial.print("position recorded");
    Serial.print("arr_in_x");
    Serial.print(out_x);
    Serial.print("arr_in_y");
    Serial.print(out_y);

    button_flag = 1;
  }

  if (blue == 0 && button_flag == 1)
  {
    button_flag = 0;
  }
    An   =   atan2 (  - currentPositiony + out_y ,  ( out_x -   currentPositionx) );
    kp = 1 ;  //0.89;
    ki =  0 ;
    kd = 10; //  5 ;
    float   error_r         =      0   -  currentPosition_R ;
    float   integral_r      =      integral_r           +  error_r;
    float   derivative_r    =      error_r              -  previous_error_r ;
    speedt                  =      kp * error_r  +  ki * integral_r + kd * derivative_r ;

    previous_error_r = error_r;
    Serial.print("\t AUTOMATIC  MODE ON");

  }

  An = An + 6.28 ;
  An =   ( An * (180 / pi) ) ;
  //    Serial.print( "   angle  ");
  //    Serial.print( An );
  An = ( An / (180 / pi)   );

  float theta1 = ( An - 0.785 );
  float theta2 = ( An - 2.355 );
  float theta3 = ( An - 3.927 );
  float theta4 = ( An - 5.495 );
  Serial.print("speed");
  Serial.println( speedt);
  motor1 =  speedt * sin ( theta1); // +  ;
  motor2 =  speedt * sin ( theta2) ; //+  ;
  motor3 =  speedt * sin ( theta3)  ;//+  ;
  motor4 =  speedt * sin ( theta4)  ;//+  ;

  // polar logic end  *********************************

  motor1 =  (motor1  + pid  );
  motor2 =  (motor2  + pid  );
  motor3 =  (motor3  + pid  );
  motor4 =  (motor4  + pid  );


  motor1 =  constrain  (  motor1 , - Max_speed  , Max_speed  )   ;
  motor2 =  constrain  (  motor2 , - Max_speed  , Max_speed )   ;
  motor3 =  constrain  (  motor3 , - Max_speed  , Max_speed  )   ;
  motor4 =  constrain  (  motor4 , - Max_speed  , Max_speed  )   ;


  //  Serial.print("      ");
  //  Serial.print(sin(pi / 3 ));
  //  Serial.print(" \t motor1pwm : ");
  //  Serial.print( motor1);
  //  //
  //  Serial.print(" \t motor2pwm : ");
  //  Serial.print(motor2);
  //  //
  //  Serial.print(" \t motor3pwm : ");
  //Serial.print(motor3);
  //
  //  Serial.print(" \t motor4pwm : ");
  //  Serial.println(motor4);

  // sabertooth comand *****************************

  ST.motor( 2 , -(motor1 ));  // motor poition 1
  ST1.motor(2 ,  (motor2   ));   // motor poition 2
  ST1.motor(1 , -(1.3 * motor3 )); // motor poition 3
  ST.motor( 1 , -(motor4));   // motor poition 4

  // sabertooth comand *****************************

  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ VECTOR ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

}

void X()
{
  if (digitalRead(encoderpinxA) != digitalRead(encoderpinxB))
  {
    currentPositionx = currentPositionx - 1;

  }
  else {
    currentPositionx = currentPositionx +  1;
  }
}

void Y()
{
  if (digitalRead(encoderpinyA) != digitalRead(encoderpinyB))
  {
    currentPositiony = currentPositiony - 1;
  }
  else
  {
    currentPositiony = currentPositiony + 1;
  }
}
