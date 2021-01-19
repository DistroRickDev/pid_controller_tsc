/********************************************************
   PID Basic Example
   Reading analog input 0 to control analog PWM output 3
 ********************************************************/
#include <PID_v1.h>
#include <Wire.h>

//Pin definition
#define PWM 3
#define DIR 2

//Some constants
#define MPU_ADDR 0x68


int16_t MIN_VAL = 265;
int16_t MAX_VAL = 402;
int16_t accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z;

static volatile uint8_t dir = 0;

double x;
double y;
double z;

int vel = 0;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 15 , Ki = 0 , Kd = 0;             // modify for optimal performance

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int current_t = 0;
int last_t = 0;
int t_limit = 200; //200 ms

void setup()
{
  Setpoint = 120;
  //init serial protocol
  Serial.begin(115200);

  Serial.println("INIT STARTED");

  //init I2C
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  //defining pin mode
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);

  digitalWrite(DIR, HIGH);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  myPID.SetOutputLimits(-100, 100); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

  Serial.println("INIT SUCCESSFUL");
  current_t = millis();
}



void loop()
{
  current_t = millis();
  if (current_t - last_t >= t_limit) {
    Setpoint = 50;
    ang_vel();
    Input = vel;
    myPID.Compute();  // calculate new output
    Serial.print("OUTPUT:"); Serial.println(Output);
    pwmOut(Output / 1.1);
    last_t = current_t;
  }

}

void read_x_angle()
{
  //Lets add 10
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  accel_x = Wire.read() << 8 | Wire.read();
  accel_y = Wire.read() << 8 | Wire.read();
  accel_z = Wire.read() << 8 | Wire.read();
  int xAng = map(accel_x, MIN_VAL, MAX_VAL, -90, 90);
  int yAng = map(accel_y, MIN_VAL, MAX_VAL, -90, 90);
  int zAng = map(accel_z, MIN_VAL, MAX_VAL, -90, 90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  Serial.print("X:");
  Serial.println(x);  
}

void ang_vel(){
  int b = millis();
  read_x_angle();
  vel = (x-0)/ (millis() - b);
  Serial.print("VEL:");
  Serial.println(vel);  
}

void pwmOut(int out) {
  if (out > 0) {                         // if REV > encoderValue motor move in forward direction.
    analogWrite(PWM, out);         // Enabling motor enable pin to reach the desire angle
    forward();                           // calling motor to move forward
  }
  else {
    analogWrite(PWM, abs(out));          // if REV < encoderValue motor move in forward direction.
    reverse();                            // calling motor to move reverse
  }
}

void forward () {
  digitalWrite(DIR , HIGH);
  //digitalWrite(DIR, LOW);
}

void reverse () {
  digitalWrite(DIR, LOW);
  //digitalWrite(DIR , HIGH);
}
