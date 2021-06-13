#include <MadgwickAHRS.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Arduino_LSM9DS1.h>
#include <Wire.h>

//----Madgwick AHRS Setup/Variables
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float aRX, aRY, aRZ; //Raw values
float gRX, gRY, gRZ;
float aX, aY, aZ; // Converted values
float gX, gY, gZ;
float roll, pitch, heading;
unsigned long microsNow;

//----PID Variables
double Pk = 2.0;
double Ik = 0.0;
double Dk = 0.0;
double Setpoint = 0.2;
double Input;
double Output;
PID PID(&Input, &Output, &Setpoint, Pk, Ik , Dk, P_ON_M , DIRECT); // PID Setup

//----Motor & Servo setup
Servo motor;
byte motorPin = 9; // signal pin for the ESC.
Servo servo;
byte servoPin = 8; // servo signal pin

int servPos = 130; // 90 degrees is level for gyro platter 5-175 range
int escSpeed = 900; // lowest signal aka off throttle 900 - 2000 pwm value

void setup() {
  Serial.begin(9600);

  motor.attach(motorPin);
  servo.attach(servoPin);

  PID.SetMode(AUTOMATIC);
  PID.SetOutputLimits(-220000, 220000);
  PID.SetSampleTime(10);

  motor.writeMicroseconds(escSpeed); // send "stop" signal to ESC. Also necessary to arm the ESC.
  servo.write(servPos);

  // Start the IMU and filter
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  filter.begin(10);

  delay(1500); // Delay to allow the ESC to recognize the stopped signal.

  escSpeed = 2000; //*NOTE* Set to 900 prevent gyro from starting automatically
  motor.writeMicroseconds(escSpeed);

  delay(3000); //Delay to allow gyro to get up to speed

  // Initialize variables to pace updates to correct rate
  microsPerReading = 1000000;
  microsPrevious = micros();
}

void loop() {
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    readIMU();

    // convert from raw data to gravity and degrees/second units
    aX = convertRawAcceleration(aRX);
    aY = convertRawAcceleration(aRY);
    aZ = convertRawAcceleration(aRZ);
    gX = convertRawGyro(gRX);
    gY = convertRawGyro(gRY);
    gZ = convertRawGyro(gRZ);

    // update the filter, which computes orientation
    filter.updateIMU(gX, gY, gZ, aX, aY, aZ);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    Input = pitch;
    PID.Compute();
    Output = constrain(Output, -35, 35);

    servPos = 130 + Output; // add or subtract roll
    motor.writeMicroseconds(escSpeed); // Send signal to ESC.

    // Increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}


/**
  Read accel and gyro data.
  returns true if value is 'new' and false if IMU is returning old cached data
*/
bool readIMU() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(aRX, aRY, aRZ);
    IMU.readGyroscope(gRX, gRY, gRZ);
    return true;
  }
  return false;
}

float convertRawAcceleration(int aRaw) {
  // since we are using 4 g range
  // -4 g maps to a raw value of -32768
  // +4 g maps to a raw value of 32767

  float a = (aRaw * 4.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 2000 degrees/seconds range
  // -2000 maps to a raw value of -32768
  // +2000 maps to a raw value of 32767

  float g = (gRaw * 2000.0) / 32768.0;
  return g;
}
