#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PID_v1.h>

#define PWM 2
#define DIR 3

Adafruit_MPU6050 mpu;

int current_millis, last_millis, time_period;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = .3 , Ki = 0 , Kd = 0;             // modify for optimal performance

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);

  //defining pin mode
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  setUpSensor();
  
  time_period = 250;
  
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  myPID.SetOutputLimits(-255, 255); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
  delay(1000);
}

void loop() {
  current_millis = millis();
  if (current_millis - last_millis >= time_period)
  {
    Setpoint = 0;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    Input = a.acceleration.y * 57.29578;
    Serial.print("Y: ");Serial.println(Input);
    myPID.Compute();  // calculate new output
    Serial.print("OUTPUT:"); Serial.println(Output);
    pwmOut(Output);
    last_millis = current_millis;
  }
}


void pwmOut(int out) {
  if (out > 0) {
    forward();// if REV > encoderValue motor move in forward direction.
    analogWrite(PWM, out);         // Enabling motor enable pin to reach the desire angle
    // calling motor to move forward
  }
  else {
    reverse();                            // calling motor to move reverse
    analogWrite(PWM, abs(out));          // if REV < encoderValue motor move in forward direction.

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

void setUpSensor()
{
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}
