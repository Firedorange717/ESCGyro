#include <PID_v1.h>
#include <Servo.h>
#include <Arduino_LSM9DS1.h>
#include <Wire.h>

double Pk = 2.0;
double Ik = 0.0;
double Dk = 0.0;

double Setpoint = 0.2;
double Input;
double Output;
PID PID1(&Input, &Output, &Setpoint, Pk, Ik , Dk, P_ON_M , DIRECT); // PID Setup


float accelX,            accelY,             accelZ,            // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX,             gyroY,              gyroZ,             // units dps (degrees per second)
      gyroRoll,          gyroPitch,          // units degrees (expect major drift)
      accRoll,           accPitch,            // units degrees (roll and pitch noisy)
      complementaryRoll, complementaryPitch;  // units degrees (excellent roll, pitch)

float Time;
unsigned long lastTime;

byte motorPin = 9; // signal pin for the ESC.
byte servoPin = 8;

Servo motor;
Servo servo;

int servPos = 130; // 90 degrees is level for gyro platter 5-175 range
int escSpeed = 900; // lowest signal aka off throttle 900 - 2000 pwm value

void setup() {
  motor.attach(motorPin);
  servo.attach(servoPin);

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-220000, 220000);
  PID1.SetSampleTime(10);

  motor.writeMicroseconds(escSpeed); // send "stop" signal to ESC. Also necessary to arm the ESC.
  servo.write(servPos);
  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  delay(1500); // delay to allow the ESC to recognize the stopped signal.

  escSpeed = 2000; //disable to prevent gyro from starting automatically
  motor.writeMicroseconds(escSpeed);

  delay(3000);

  lastTime = millis();
}

void loop() {
  readIMU();
  calcAngle();

  Input = complementaryPitch;

  PID1.Compute();
  Output = constrain(Output, -35, 35);

  servPos = 130 + Output; // add or subtract roll
  //servo.write(servPos); // move servo

  motor.writeMicroseconds(escSpeed); // Send signal to ESC.

  Serial.print(Output);
  Serial.print(',');
  Serial.print(accRoll);
  Serial.print(',');
  Serial.print(gyroRoll);
  Serial.print(',');
  Serial.print(complementaryRoll);
  Serial.print(',');
  Serial.println(servPos);

  delay(10);
}

void calcAngle() {
  accRoll = -atan2(accelX / 9.8, accelZ / 9.8) / 2 / 3.141592654 * 360;
  accPitch = -atan2(accelY / 9.8, accelZ / 9.8) / 2 / 3.141592654 * 360;

  Time = (millis() - lastTime) / 1000;
  lastTime = millis();

  gyroRoll = gyroRoll + (gyroX / lastTime);
  gyroPitch = gyroPitch + (gyroY / lastTime);

  complementaryRoll = 0.95 * gyroRoll + 0.05 * accRoll;
  complementaryPitch = 0.95 * gyroPitch + 0.05 * accPitch;  // Complementary filter favors gyro short term acccelerometer long term
}

/**
  Read accel and gyro data.
  returns true if value is 'new' and false if IMU is returning old cached data
*/
bool readIMU() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    return true;
  }
  return false;
}
