/*Example program for the openIMU
 The sensors used in this example are the ADXL345, the L3G, the LSM303DLHC, and the BMP085
 If the device is moved during startup it will not function properly. 
 
 Copyright (C) 2012  Michael Baker
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 
 Gyro must be in RAD/s
 Sensors must be in the North East Down convention
 This example code will only work on the MEGA
 To use on a different arduino change the slave select defines or use digitalWrite 
 */
#include <Wire.h>
#include <SPI.h>
#include <openIMU.h>

//general SPI defines
#define READ 0x80
#define WRITE 0x00
#define MULTI 0x40
#define SINGLE 0x00

//gyro defines - ST L3G2
#define L3G_CTRL_REG1 0x20
#define L3G_CTRL_REG2 0x21
#define L3G_CTRL_REG3 0x22
#define L3G_CTRL_REG4 0x23
#define L3G_CTRL_REG5 0x24
#define L3G_OUT_X_L 0x28

//acc defines - Analog Devices ADXL345
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32
//use calibrate_acc to find these values
//take the max and min from each axis
//then use these formulas
//ACC_SC_etc_NEG = 9.8 / axis min
//ACC_SC_etc_POS = 9.8 / axis max
//gravity is sensed differently in each direction
//that is why datasheets list a range of LSB/g
//the values from the datasheets can be used but it will hurt the performance of the filter
//these calibration values will change over time due to factors like tempature
#define ACC_SC_X_NEG 0.038431372f
#define ACC_SC_X_POS 0.036029411f
#define ACC_SC_Y_NEG 0.035897435f
#define ACC_SC_Y_POS 0.03828125f
#define ACC_SC_Z_NEG 0.037984496f
#define ACC_SC_Z_POS 0.039516129f

//mag defines ST LSM303DLHC - will possibly work with the HMC5883L
#define MAG_ADDRESS 0x1E
#define LSM303_CRA_REG (uint8_t)0x00 //??? Wire.h needs to be fixed
#define LSM303_CRB_REG 0x01
#define LSM303_MR_REG 0x02
#define LSM303_OUT_X_H 0x03
//use calibrate_mag to find these values
#define compassXMax 301.0f
#define compassXMin -316.0f
#define compassYMax 135.0f
#define compassYMin -451.0f
#define compassZMax 387.0f
#define compassZMin -182.0f
#define inverseXRange (float)(2.0 / (compassXMax - compassXMin))
#define inverseYRange (float)(2.0 / (compassYMax - compassYMin))
#define inverseZRange (float)(2.0 / (compassZMax - compassZMin))

//barometer defines
//the code for the BMP085 uses the data ready interrupt so the program isn't blocked 
#define BMP085_ADDRESS 0x77
#define OSS 0x00
#define READY_PIN 10
#define POLL_RATE 0

//using these macros in place of digitalWrite is much faster
//however digitalWrite will work when using SPI 
#define GyroSSOutput() DDRL |= 1<<0 //this is the same as pinMode(49,OUTPUT)
#define GyroSSHigh() PORTL |= 1<<0 //this is like digitalWrite(49,HIGH) but faster
#define GyroSSLow() PORTL &= ~(1<<0)

#define AccSSOutput() DDRL |= 1<<1 //this is the same as pinMode(49,OUTPUT)
#define AccSSHigh() PORTL |= 1<<1 //this is like digitalWrite(49,HIGH) but faster
#define AccSSLow() PORTL &= ~(1<<1)


typedef union{
  struct{
    int16_t x;
    int16_t y;
    int16_t z;
  }
  v;
  uint8_t buffer[6];
}
Sensor_t;

Sensor_t gyro;
Sensor_t acc;
Sensor_t mag;

//barometer variables
int pressureState;
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
unsigned char msb;
unsigned char lsb;
unsigned char xlsb;
long x1;
long x2;
long x3;
long b3;
long b5;
long b6;
long p;
unsigned long b4;
unsigned long b7;
unsigned int ut;
unsigned long up;
unsigned long baroTimer;
float pressureRatio;
long pressure;
short temperature;
boolean newBaro;
int baroCount;
float baroSum;
long pressureInitial; 
float rawAltitude;


//for the calibration of the gyro
int32_t gyroSumX,gyroSumY,gyroSumZ;
int16_t offsetX,offsetY,offsetZ;

float radianGyroX,radianGyroY,radianGyroZ;
float floatMagX,floatMagY,floatMagZ;//needs to be a float so the vector can be normalized
int16_t rawX,rawY,rawZ;
float scaledAccX,scaledAccY,scaledAccZ;
float smoothAccX,smoothAccY,smoothAccZ;
float accToFilterX,accToFilterY,accToFilterZ;
float dt;


uint8_t loopCount;
uint16_t i;//index for buffering in the data
uint16_t j;
uint32_t timer,printTimer;

//this is how you use the AHRS and Altimeter
openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&scaledAccX,&scaledAccY,&scaledAccZ,&floatMagX,&floatMagY,&floatMagZ,&rawAltitude,&dt);
//this is how to use the full AHRS 
//the varialbes are passed by reference don't forget the &
//openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&floatMagX,&floatMagY,&floatMagZ,&dt);
//this is how to use the IMU
//openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&smoothAccX,&smoothAccY,&smoothAccZ,&dt);

void setup(){
  Serial.begin(115200);
  Serial.println("Keeping the device still and level during startup will yeild the best results");
  Wire.begin();
  TWBR = ((F_CPU / 400000) - 16) / 2;//set the I2C speed to 400KHz - why isn't this an option in Wire.h?????
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);  
  
  AccSSOutput();//this was moved from the init
  AccSSHigh();//if high isn't written to both devices befor config 
  GyroSSOutput();//the SPI bus will be addressing both devices 
  GyroSSHigh();
  GyroInit();
  
  Serial.println("Gyro init complete");
  AccInit();
  Serial.println("Accelerometer init complete");
  MagInit();
  Serial.println("Compass init complete");
  GetGyro();
  GetAcc();
  GetMag();
  imu.InitialQuat();
  Serial.println("Initial quaternion has been calculated");
  BaroInit();
  Serial.println("Barometer has been initialized");
  loopCount = 0;
  printTimer = millis();
  timer = micros();
}

void loop(){
  PollPressure();
  if (newBaro == true){
    newBaro = false;
    GetAltitude(&pressure,&pressureInitial,&rawAltitude);
    imu.BaroKalUpdate();
  }  

  if (micros() - timer >= 5000){  
    //example for the full AHRS update
    //on the MEGA 2560 this takes about 4.1ms
    //using I2C devices will increase the execution time
    dt = ((micros() - timer) / 1000000.0);
    timer = micros();
    GetGyro();
    GetAcc();
    GetMag();
    imu.AHRSupdate();
  }
  /*  if (micros() - timer >= 5000){   
   //example for the full AHRS update
   //on the MEGA 2560 this takes about 3.6ms
   //using I2C devices will increase the execution time
   dt = ((micros() - timer) / 1000000.0);
   timer = micros();
   GetGyro();
   GetAcc();
   GetMag();
   imu.AHRSupdate();
   }*/
  /*if (micros() - timer >= 2500){
   //example for the pitch and roll calculations only
   //on the MEGA 2560 this takes about 1.7ms
   dt = ((micros() - timer) / 1000000.0);
   timer = micros();
   GetGyro();
   GetAcc();
   imu.IMUupdate();
   }*/

  if (millis() - printTimer > 20){
    printTimer = millis();
    imu.GetEuler();
    Serial.print(printTimer);
    Serial.print(",");
    Serial.print(imu.pitch);
    Serial.print(",");
    Serial.print(imu.roll);
    Serial.print(",");
    Serial.print(imu.yaw);
    Serial.print(",");
    Serial.println(imu.altitude); 
  }
}

void Smoothing(int16_t *raw, float *smooth){
  *smooth = (*raw * (0.15)) + (*smooth * 0.85);
}



















