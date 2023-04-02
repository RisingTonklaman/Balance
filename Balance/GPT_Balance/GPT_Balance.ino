#include <Wire.h>
#include <PID_v1.h>
#include <MPU6050.h>

MPU6050 mpu;

double Setpoint, Input, Output;
double Kp = 1, Ki = 0.1, Kd = 0.05;
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Wire.begin();
  mpu.initialize();
  
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255); // motor speed limits
}

void loop() {
  // Read sensor data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate angle and angular velocity
  double angle = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  double gyroRate = (double)gy / 131.0;
  
  // Set the PID setpoint to the desired angle (e.g., 0 degrees)
  Setpoint = 0;
  
  // Update the PID controller with the current angle and angular velocity
  Input = angle;
  pid.Compute();
  
  // Adjust the motor speeds based on the PID output
  int motorSpeed = (int)Output;
  // Code to adjust the motor speeds based on the PID output goes here...
}
