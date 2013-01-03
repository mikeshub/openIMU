/*Example program for the openIMU
 The sensors used are the ADXL345, the L3G, and the LSM303DLHC
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

//mag defines ST LSM303DLHC - will possibly work with the HMC5883L
#define MAG_ADDRESS 0x1E
#define LSM303_CRA_REG (uint8_t)0x00 //??? Wire.h needs to be fixed
#define LSM303_CRB_REG 0x01
#define LSM303_MR_REG 0x02
#define LSM303_OUT_X_H 0x03
#define compassXMax 224.0f
#define compassXMin -377.0f
#define compassYMax 148.0f
#define compassYMin -459.0f
#define compassZMax 364.0f
#define compassZMin -208.0f
#define inverseXRange (float)(2.0 / (compassXMax - compassXMin))
#define inverseYRange (float)(2.0 / (compassYMax - compassYMin))
#define inverseZRange (float)(2.0 / (compassZMax - compassZMin))

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

//for the calibration of the gyro
int32_t gyroSumX,gyroSumY,gyroSumZ;
int16_t offsetX,offsetY,offsetZ;

float radianGyroX,radianGyroY,radianGyroZ;
float floatMagX,floatMagY,floatMagZ;//needs to be a float so the vector can be normalized
float smoothAccX,smoothAccY,smoothAccZ;
float accToFilterX,accToFilterY,accToFilterZ;
float dt;


uint8_t loopCount;
uint16_t i;//index for buffering in the data
uint16_t j;
uint32_t timer,printTimer;
//this is how to use the full AHRS 
//the varialbes are passed by reference don't forget the &
openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&floatMagX,&floatMagY,&floatMagZ,&dt);
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
  printTimer = millis();
  timer = micros();
}

void loop(){
  if (micros() - timer >= 5000){   
    //example for the full AHRS update
    //on the MEGA 2560 this takes about 3.6ms
    //using I2C devices will increase the execution time
    dt = ((micros() - timer) / 1000000.0);
    timer = micros();
    GetGyro();
    GetAcc();
    GetMag();
    imu.AHRSupdate();
  }
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
    Serial.println(imu.yaw);
  }


}

void MagInit(){
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(LSM303_CRA_REG);
  Wire.write(0x1C);//220Hz update rate
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(LSM303_CRB_REG);
  Wire.write(0x60);//+/- 2.5 gauss
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(LSM303_MR_REG);
  Wire.write((uint8_t)0x00);//continuous conversion mode
  Wire.endTransmission();
}

void AccInit(){
  SPI.setDataMode(SPI_MODE3);

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | BW_RATE);
  SPI.transfer(0x0C);//400hz
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | POWER_CTL);
  SPI.transfer(0x08);//start measurment
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | DATA_FORMAT);
  SPI.transfer(0x0B);//full resolution + / - 16g
  AccSSHigh();

  for(j = 0; j < 300; j++){
    GetAcc();//to get the smoothing filters caugt up
    delay(5);
  }
}

void GyroInit(){
  SPI.setDataMode(SPI_MODE0);

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG1 | WRITE | SINGLE);
  SPI.transfer(0xCF); //fastest update rate 30Hz cutoff
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG2 | WRITE | SINGLE);
  SPI.transfer(0x00); //high pass filter disabled
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG3 | WRITE | SINGLE);
  SPI.transfer(0x00); //not using interrupts
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG4 | WRITE | SINGLE);
  SPI.transfer(0x20); //2000dps scale
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG5 | WRITE | SINGLE);
  SPI.transfer(0x02); //out select to use the second LPF
  GyroSSHigh();

  //this section takes an average of 500 samples to calculate the offset
  //if this step is skipped the IMU will still work, but this simple step gives better results
  offsetX = 0;
  offsetY = 0;
  offsetZ = 0;
  for (j = 0; j < 500; j ++){//give the internal LPF time to warm up
    GetGyro();
    delay(3);
  }
  for (j = 0; j < 500; j ++){//give the internal LPF time to warm up
    GetGyro();
    gyroSumX += gyro.v.x;
    gyroSumY += gyro.v.y;
    gyroSumZ += gyro.v.z;
    delay(3);
  }
  offsetX = gyroSumX / 500.0;
  offsetY = gyroSumY / 500.0;
  offsetZ = gyroSumZ / 500.0;
}

void GetMag(){
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(LSM303_OUT_X_H);
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDRESS, 6);
  //the arudino is a little endian system, but the compass is big endian
  mag.buffer[1] = Wire.read();//X
  mag.buffer[0] = Wire.read();
  mag.buffer[5] = Wire.read();//Z
  mag.buffer[4] = Wire.read();
  mag.buffer[3] = Wire.read();//Y
  mag.buffer[2] = Wire.read();


  floatMagX = ((float)mag.v.x - compassXMin) * inverseXRange - 1.0;
  floatMagY = ((float)mag.v.y - compassYMin) * inverseYRange - 1.0;
  floatMagZ = ((float)mag.v.z - compassZMin) * inverseZRange - 1.0;

}

void GetGyro(){
  SPI.setDataMode(SPI_MODE0);
  GyroSSLow();
  SPI.transfer(L3G_OUT_X_L  | READ | MULTI);
  for (i = 0; i < 6; i++){//the endianness matches as does the axis order
    gyro.buffer[i] = SPI.transfer(0x00);
  }
  GyroSSHigh();
  //don't forget to convert to radians per second. This absolutely will not work otherwise
  //check the data sheet for more info on this
  radianGyroX = ToRad((gyro.v.x - offsetX) * 0.07);
  radianGyroY = ToRad((gyro.v.y - offsetY) * 0.07);
  radianGyroZ = ToRad((gyro.v.z - offsetZ) * 0.07);
}

void GetAcc(){
  static int16_t negated;
  SPI.setDataMode(SPI_MODE3);
  AccSSLow();
  SPI.transfer(DATAX0 | READ | MULTI);
  for (i = 0; i < 6; i++){//the endianness matches as does the axis order
    acc.buffer[i] = SPI.transfer(0x00);
  }
  AccSSHigh();  

  //the filter expects gravity to be in NED coordinates
  //switching the sign will fix this
  //the raw values are not negated because the accelerometer values will be used with a barometer for altimeter data in a future revisi

  /*
  negated = acc.v.x * -1;
   Smoothing(&negated,&smoothAccX);//this is a very simple low pass digital filter
   negated = acc.v.y * -1;
   Smoothing(&negated,&smoothAccY);//it helps significiantlly with vibrations. 
   negated = acc.v.z * -1;
   Smoothing(&negated,&smoothAccZ);//if the applicaion is not prone to vibrations this can skipped and the raw value simply recast as a float
   */
  //the order and signs have been switched due to the accelerometer being mounted off by 90 degrees
  //one must be careful to make sure that all of the sensors are in the North, East, Down convention
  negated = acc.v.y * -1;
  Smoothing(&negated,&smoothAccX);//this is a very simple low pass digital filter
  negated = acc.v.x;
  Smoothing(&negated,&smoothAccY);//it helps significiantlly with vibrations. 
  negated = acc.v.z * -1;
  Smoothing(&negated,&smoothAccZ);//if the applicaion is not prone to vibrations this can skipped and the raw value simply recast as a float
  accToFilterX = smoothAccX;//if the value from the smoothing filter is sent it will not work when the algorithm normalizes the vector
  accToFilterY = smoothAccY;
  accToFilterZ = smoothAccZ;
}

void Smoothing(int16_t *raw, float *smooth){
  *smooth = (*raw * (0.15)) + (*smooth * 0.85);
}















