
// Includes
#include <Arduino.h>

#include <LSM6DSOSensor.h>  // Accelerometer + Gyro
#include <LIS2DW12Sensor.h> // Accelerometer
#include <LIS2MDLSensor.h>  // Magnetometer

#include <iostream>
#include <math.h>
#include "six_axis_comp_filter.h"
using namespace std;

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

// Macros and Globals
#define PI 3.1415926f
#define HALF_PI 1.5707963f
#define TWO_PI 6.2831853f
#define SQRE(x) ((x) * (x))

// Components
LSM6DSOSensor *AccGyr;
LIS2DW12Sensor *Acc2;
LIS2MDLSensor *Mag;

// Variablen
int delay1 = 50;
float delay2 = static_cast<float>(delay1) / 1000; // s
float tau1 = 0.3;                                 // s

// Class CompSixAxis global erstellen (Funktioniert nicht im void setup() bei Arduino)
CompSixAxis CompSixAxis1(delay2, tau1);

void setup()
{
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();

  AccGyr = new LSM6DSOSensor(&DEV_I2C);
  AccGyr->begin();
  AccGyr->Enable_X();
  AccGyr->Enable_G();
  Acc2 = new LIS2DW12Sensor(&DEV_I2C);
  Acc2->Enable_X();
  Mag = new LIS2MDLSensor(&DEV_I2C);
  Mag->Enable();

  //  CompSixAxis1.CompAccelUpdate(0, 0, 0);
  //  CompSixAxis1.CompStart();
}

void loop()
{

  delay(delay1);

  // Read accelerometer and gyroscope
  int32_t accelerometer[3];
  int32_t gyroscope[3];
  AccGyr->Get_X_Axes(accelerometer);
  AccGyr->Get_G_Axes(gyroscope);

  // Read accelerometer
  int32_t accelerometer2[3];
  Acc2->Get_X_Axes(accelerometer2);

  // Read magnetometer
  int32_t magnetometer[3];
  Mag->GetAxes(magnetometer);

  float acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, AngXout, AngYout;

  // Sensorwerte Accelerometer in float
  acc_x = static_cast<float>(accelerometer[0]) / 1000 * 9.81; // mg to m/s^2
  acc_y = static_cast<float>(accelerometer[1]) / 1000 * 9.81;
  acc_z = static_cast<float>(accelerometer[2]) / 1000 * 9.81;
  // Sensorwerte Gyroscope in float
  gyr_x = static_cast<float>(gyroscope[0]) / 1000 / 360 * 2 * PI; // mdps to rad/s
  gyr_y = static_cast<float>(gyroscope[1]) / 1000 / 360 * 2 * PI;
  gyr_z = static_cast<float>(gyroscope[2]) / 1000 / 360 * 2 * PI;

  CompSixAxis1.CompAccelUpdate(acc_x, acc_y, acc_z);
  CompSixAxis1.CompGyroUpdate(gyr_x, gyr_y, gyr_z);
  CompSixAxis1.CompUpdate();
  CompSixAxis1.CompAnglesGet(&AngXout, &AngYout);

  //*********************************************************************************
  // Serial Output für Visualisierung am PC
  //*********************************************************************************
  // Output data
  SerialPort.print(accelerometer[0]); // Acc[mg] [0]
  SerialPort.print(",");
  SerialPort.print(accelerometer[1]); // Acc[mg] [1]
  SerialPort.print(",");
  SerialPort.print(accelerometer[2]); // Acc[mg] [2]
  SerialPort.print(",");
  SerialPort.print(gyroscope[0]); // Gyr[mdps] [0]
  SerialPort.print(",");
  SerialPort.print(gyroscope[1]); // Gyr[mdps] [1]
  SerialPort.print(",");
  SerialPort.print(gyroscope[2]); // Gyr[mdps] [2]
  SerialPort.print(",");
  // Umgewandelte Werte
  SerialPort.print(acc_x);
  SerialPort.print(",");
  SerialPort.print(acc_y);
  SerialPort.print(",");
  SerialPort.print(acc_z);
  SerialPort.print(",");
  SerialPort.print(gyr_x);
  SerialPort.print(",");
  SerialPort.print(gyr_y);
  SerialPort.print(",");
  SerialPort.print(gyr_z);
  // Roll / Pitch
  SerialPort.print(",");
  SerialPort.print(AngXout / 2 / PI * 360);
  SerialPort.print(",");
  SerialPort.print(AngYout / 2 / PI * 360);
  SerialPort.print("\r\n"); // new line und carriage return
}
