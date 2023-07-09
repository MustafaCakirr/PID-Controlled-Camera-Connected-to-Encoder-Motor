#include <PID_v1.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
double degree = 0;
//MC///////////////////////////////////////////////////////
unsigned long ilk_zaman = 0;
unsigned long son_zaman = 0;
unsigned long prev_time = 0;
unsigned long current_time;

#define camera 14
unsigned long cam_current;
unsigned long cam_prev = 0;
int cam_a = 0;
int eq = 0;

File my_file;
const int chipSelect = BUILTIN_SDCARD;

Adafruit_BNO055 myIMU = Adafruit_BNO055(-1, 0x28);
float gyro_x;
float gyro_z;
float gyro_y;
float acc_x;
float acc_y;
float acc_z;
float mag_x;
float mag_y;
float mag_z;
float temp;
float Alti;

imu::Vector<3> euler;
imu::Vector<3> acc;
imu::Vector<3> gyro;
imu::Vector<3> mag;
imu::Vector<3> lineer;

#define encodPinA1 34
#define encodPinB1 35
#define M1 37
#define M2 36

double pitchy;
//double kp = 15, ki = 1.25, kd = 0.025;
//double kp = 19, ki =1, kd = 0.055;    ideale yakın
double kp = 19, ki = 1.3, kd = 0.040;
double input = 0, outputt = 0, setpoint = 0;
volatile long encoderPos = 0;
PID myPID(&input, &outputt, &setpoint, kp, ki, kd, DIRECT);
int gyrooperational;
int prevgyro = 0;
int counter = 0;
float a;
float error_point;

void setup(void)
{
  Serial1.begin(115200);
  pinMode(buzzer_pin, OUTPUT);
  myIMU.begin(0x0C);
  myIMU.setExtCrystalUse(true);
  pinMode(encodPinA1, INPUT_PULLUP);
  pinMode(encodPinB1, INPUT_PULLUP);
  attachInterrupt(34, encoder, FALLING);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  digitalWrite(buzzer_pin, HIGH);
  delay(1000);
  digitalWrite(buzzer_pin, LOW);
  pinMode(camera, OUTPUT);

  void imu()
  {
    acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    //lineer = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    gyro.x();
    gyro.y();
    gyro.z();
    acc.x();
    acc.y();
    acc.z();
    mag.x();
    mag.y();
    mag.z();
  }

  void PID()
  {
    imu();
    if (current_time - prev_time > 25) {
      gyrooperational = euler.x();
      if (prevgyro - gyrooperational > 200) {
        counter++;
      }
      if (prevgyro - gyrooperational < -200) {
        counter--;
      }
      a = (counter * 360 + gyrooperational);

      prevgyro = euler.x();
      prev_time = current_time;
    }
    error_point = outputt / kp;

    setpoint = (a) / 3;

    input = encoderPos;

    myPID.Compute();

    pwmOut(outputt);
  }

  void pwmOut(int out)
  {
    if (out > 0) {
      analogWrite(M1, 0);
      analogWrite(M2, out);
    } else if (out < 0) {
      analogWrite(M1, abs(out));
      analogWrite(M2, 0);
    } else {
      analogWrite(M1, 0);
      analogWrite(M2, 0);
    }
  }

  void encoder()
  {
    if (digitalRead(encodPinB1) == HIGH) {
      encoderPos++;

    } else {
      encoderPos--;
    }
  }


  void loop()
  {
    PID();
  }
