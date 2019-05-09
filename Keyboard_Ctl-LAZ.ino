
//Draft of
//control of the dc motors && Ultrasonic sensor
//from keyboard input
#include <SoftwareSerial.h>

SoftwareSerial BlueTooth(6,13); //6 RXD, 13 TXD

 
const int trigPin = 9;
const int echoPin = 10;
long duration;
double distance;
String cmd;

//Arduino pins for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

/*8-bit bus after the 74HC595 shift register 
  (not Arduino pins)
  These are used to set the direction of the bridge driver..*/
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4


//Arduino pins for PWM signals, pins 11 and 3 are PWM pins
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3


//Codes for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

void setup() 
{
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  Serial.begin(9600);
  BlueTooth.begin(9600);
  Serial.println("Enter a valid command: ");
}

//this returns the distance read by the sensor
double getDistance()
{
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  duration = pulseIn(echoPin,HIGH);
  distance = duration*0.034/2;
  return distance;
}

//stops the robot if conditions are met
void backUp()
{
  motor(1,BRAKE,0);
  motor(2,BRAKE,0);
}

//sets the speed, direction of motor 1 or 2
void motor(int nMotor, int direction_, int speed)
{
  int motorPositive, motorNegative;

  if (nMotor == 1 || nMotor == 2)
  {  
    switch (nMotor)
    {
      case 1:
        motorPositive   = MOTOR1_A;
        motorNegative   = MOTOR1_B;
        break;
      case 2:
        motorPositive   = MOTOR2_A;
        motorNegative   = MOTOR2_B;
        break;
      default:
        break;
    }
    
    switch (direction_)
    {
      case FORWARD:
        motor_output (motorPositive, HIGH, speed);
        motor_output (motorNegative, LOW, -1);   // -1: no PWM set
        break;
      case BACKWARD:
        motor_output (motorPositive, LOW, speed);
        motor_output (motorNegative, HIGH, -1);    // -1: no PWM set
        break;  
      case BRAKE:
      /*The AdaFruit library didn't implement a brake.
      The L293D motor driver ic doesn't have a good
      brake anyway.
      It uses transistors inside, and not mosfets.
      Some use a software break, by using a short 
      reverse voltage.
      This brake will try to brake, by enabling 
      the output and by pulling both outputs to ground.
      But it isn't a good break..*/
        motor_output (motorPositive, LOW, 255); //255: fully on.
        motor_output (motorNegative, LOW, -1);  //-1: no PWM set
        break;
      case RELEASE:
        motor_output (motorPositive, LOW, 0);  //0: output floating.
        motor_output (motorNegative, LOW, -1); //-1: no PWM set
        break;
      default:
        break;
    }
  }
}


void motor_output (int output, int high_low, int speed)
/*The function motor_ouput uses the motor driver to
  drive normal outputs like lights, relays, solenoids, 
  DC motors (but not in reverse).
  It is also used as an internal helper function 
  for the motor() function...*/
{
  int motorPWM;

  switch (output)
  {
    case MOTOR1_A:
    case MOTOR1_B:
      motorPWM = MOTOR1_PWM;
      break;
    case MOTOR2_A:
    case MOTOR2_B:
      motorPWM = MOTOR2_PWM;
      break;//
    default:
    //Use speed as error flag, -3333 = invalid output.
      speed = -3333;
      break;
  }
    if (speed != -3333)
  {
    /*Set the direction with the shift register 
      on the MotorShield, even if the speed = -1.
      In that case the direction will be set, but
      not the PWM...*/
    shiftWrite(output, high_low);
    
    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)    
    {
      analogWrite(motorPWM, speed);
    }
  }
}

/*The parameters are just like digitalWrite().
  
  The output is the pin 0...7 (the pin behind  the shift register).
  The second parameter is HIGH or LOW.

  There is no initialization function.
  Initialization is automatically done at the first
  time it is used...*/
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  /*Do the initialization on the fly, 
    at the first time it is used...*/
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    //Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    //Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    //start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  //The defines HIGH and LOW are 1 and 0.
  //So this is valid.
  bitWrite(latch_copy, output, high_low);
  /*Use the default Arduino 'shiftOut()' function to
  shift the bits with the MOTORCLK as clock pulse.
  The 74HC595 shiftregister wants the MSB first.
  After that, generate a latch pulse with MOTORLATCH...*/
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    //For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    //For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}

void goForward(int dc_mot1, int dc_mot2, int Speed)
/* sets motor1 and motor 2 and the speed forward
   since we only need the bot to stop when instructed to 
   or when it faces an obstacle,
   hence no duration needed for this function */
{
 int speed_set = 0;
 for (int i = 0; i<Speed; i++)
 {
    speed_set++;
    motor(dc_mot1,FORWARD,speed_set);
    motor(dc_mot2,FORWARD,speed_set);
    if(speed_set == Speed)
      speed_set--;
 }
}

void goBackward(int dc_mot1, int dc_mot2, int Speed)
//sets motor1 and motor 2 and the speed backward
{
 int speed_set = 0;
 for (int i = 0; i<Speed; i++)
 {
    speed_set++;
    motor(dc_mot1,BACKWARD,speed_set);
    motor(dc_mot2,BACKWARD,speed_set);
    if(speed_set == Speed)
      speed_set--;
 }
}

void RightTurn(int dc_mot1, int dc_mot2,double Speed)
//makes right turn
{
  motor(dc_mot1,FORWARD,Speed);
  motor(dc_mot2,BRAKE,0);
}

void LeftTurn(int dc_mot1, int dc_mot2,int Speed)
//makes left turn
{
  motor(dc_mot1,BRAKE,0);
  motor(dc_mot2,FORWARD,Speed); 
}

void turn()
{
   delay(1000); 
   backUp();
}

//gets command from keyboard
int getCommand()
{
  while (BlueTooth.available())
  {
    char rx = BlueTooth.read();
    cmd += rx;
    if (rx == 'k' || rx == 't' || rx == 'd')
    {
      return 1;
    }
  }
  return 0;
}

//set the direction of the mvt
int getMove()
{
  int mov = -1;
  
  if(cmd == "abort")
    mov = 0;
  else if(cmd == "left")
    mov = 1;
  else if(cmd == "right")
    mov = 2;
  else if(cmd == "back")
    mov = 3;
  else if(cmd == "forward")
    mov = 4;
        
  return mov;  
}

/*...*/
void setmotion(int mvt)
{
  switch(mvt)
  {
    case 0:
      backUp();
      break;
    case 1: //left motion
      LeftTurn(1,2,123);
      turn();
      break;
    case 2: //right motion
      RightTurn(1,2,123);
      turn();
      break; 
    case 3: //backward motion
      goBackward(1,2,255);
      break;
    case 4: //forward motion
      goForward(1,2,255); 
      break;
    default:
      break;
  }
}
/*....
......*/
void loop() 
{
  double dist = getDistance();
  //Serial.println(dist); //not needed
  //only to print the distance
  //read from sensor 
  if(dist >= 20) 
  /*the operating distance should be
  10cm far from any obstacles */
  {
    int ret = getCommand();
    if ( ret == 1 )
    {
      Serial.print(cmd);
      int movement = getMove();
      Serial.println(movement);
      if ( movement >= 0 )
      {
        setmotion(movement);
      }
      cmd = "";
    }
  }
  else
  {
    backUp();
  }
}
