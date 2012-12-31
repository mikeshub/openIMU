void GetAltitude(long *press,long *pressInit, float *alti){
  pressureRatio = (float) *press / (float) *pressInit;
  *alti = (1.0f - pow(pressureRatio, 0.190295f)) * 44330.0f;
}

void PollPressure(void){
  if (millis() - baroTimer > POLL_RATE){
    switch (pressureState){
    case 0://read ut
      StartUT();
      pressureState = 1;
      break;
    case 1://wait for ready signal
      if (digitalRead(READY_PIN) == 1){
        pressureState = 2;
        ut = ReadUT();
      }
      break;
    case 2://read up
      StartUP();
      pressureState = 3;
      break;
    case 3://wait for ready signal
      if (digitalRead(READY_PIN) == 1){
        pressureState = 4;
        up = ReadUP();
      }
      break;
    case 4://
      temperature = Temperature(ut);
      pressure = Pressure(up);
      pressureState = 0;
      newBaro = true;
      baroTimer = millis();
      break;
    }
  }
}

long Pressure(unsigned long up){


  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  return p;
}

short Temperature(unsigned int ut){

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);
}

void StartUT(void){
  //I2c.write(BMP085_ADDRESS,0xF4,0x2E);
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
}

unsigned int ReadUT(void){



  /*I2c.read(BMP085_ADDRESS,0xF6,2);
   msb = Wire.read();
   lsb = Wire.read();*/
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 2);

  msb = Wire.read();
  lsb = Wire.read();


  return ((msb << 8) | lsb);
}

void StartUP(void){
  //I2c.write(BMP085_ADDRESS,0xF4,(0x34 + (OSS<<6)));
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write((0x34 + (OSS<<6)));
  Wire.endTransmission();
}

unsigned ReadUP(void){

  //I2c.read(BMP085_ADDRESS,0xF6,3);
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);  
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();

  return ((((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS));
}

void BaroInit(void){
  pinMode(READY_PIN,INPUT);
  pressureState = 0;
  baroTimer = millis();
  newBaro = false;

  //I2c.read(BMP085_ADDRESS,0xAA,22);
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xAA);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 22);  

  msb = Wire.read();
  lsb = Wire.read();
  ac1 = (msb << 8) | lsb;

  msb = Wire.read();
  lsb = Wire.read();
  ac2 = (msb << 8) | lsb;

  msb = Wire.read();
  lsb = Wire.read();
  ac3 = (msb << 8) | lsb;

  msb = Wire.read();
  lsb = Wire.read();
  ac4 = (msb << 8) | lsb;

  msb = Wire.read();
  lsb = Wire.read();
  ac5 = (msb << 8) | lsb;

  msb = Wire.read();
  lsb = Wire.read();
  ac6 = (msb << 8) | lsb;

  msb = Wire.read();
  lsb = Wire.read();
  b1 = (msb << 8) | lsb;

  msb = Wire.read();
  lsb = Wire.read();
  b2 = (msb << 8) | lsb;

  msb = Wire.read();
  lsb = Wire.read();
  mb = (msb << 8) | lsb;

  msb = Wire.read();
  lsb = Wire.read();
  mc = (msb << 8) | lsb;

  msb = Wire.read();
  lsb = Wire.read();
  md = (msb << 8) | lsb;
  while (newBaro == false){
    PollPressure();
  }
  newBaro = false;

  //this is to get the ground pressure for relative altitude
  //lower pressure than this means positive altitude
  //higher pressure than this means negative altitude
  baroCount = 0;
  while (baroCount < 50){//use a while instead of a for loop because the for loop runs too fast
    PollPressure();
    if (newBaro == true){
      newBaro = false;
      baroCount++;
      baroSum += pressure;
    }    
  }
  pressureInitial = baroSum / 50;    
  //use the line below for altitdue above sea level
  //pressureInitial = 101325;

}



