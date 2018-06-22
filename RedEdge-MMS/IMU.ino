bool IMUSteup(void){

if(I2CwriteByte(MPU9250_ADDRESS,29,0x06))// Set accelerometers low pass filter at 5Hz
  if(I2CwriteByte(MPU9250_ADDRESS,26,0x06))// Set gyroscope low pass filter at 5Hz
    if(I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS))// Configure gyroscope range
      if(I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G))// Configure accelerometers range
        if(I2CwriteByte(MPU9250_ADDRESS,0x37,0x02))// Set by pass mode for the magnetometers
          if(I2CwriteByte(MAG_ADDRESS,0x0A,0x16))// Request continuous magnetometer measurements in 16 bits
return true;
}

void ReadAccelGyro(int& Gyro, float& Accel){
// ::: accelerometer and gyroscope :::
// Read accelerometer and gyroscope

}

void getRollPitchYaw(float& Roll,float& Pitch){
  int Gxyz[3]={0};
  float Axyz[3]={0.0};
  uint8_t Buf[14];
  float G_cal_x=0;
  float G_cal_y=0;
  float angle_x=0;
  float angle_y=0;
  float d_angle_x=0;
  float d_angle_y=0;
  float Roll_output=0;
  float Pitch_output=0;
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  // Accelerometer
  Axyz[0] = (float)(Buf[0]<<8 | Buf[1])/16384;
  Axyz[1] = (float)(Buf[2]<<8 | Buf[3])/16384;
  Axyz[2] = (float)(Buf[4]<<8 | Buf[5])/16384;
  // Gyroscope
  Gxyz[0] = (int)(Buf[8]<<8  | Buf[9]);
  Gxyz[1] = (int)(Buf[10]<<8 | Buf[11]);
  Gxyz[2] = (int16_t)(Buf[12]<<8 | Buf[13]);
  //gyro to roll and pitch calculation d_angle_x and d_angle_y are roll and pitch
  Gxyz[0] = Gxyz[0] - G_cal_x;
  Gxyz[1] = Gxyz[1] - G_cal_y;
 //here is the sample rate and 131 value which is 0.000000954 it seem
 //when you call accel function as well you need to multiply it by 2
  angle_x += Gxyz[0]*(0.000000954*2);
  angle_y += Gxyz[1]*(0.000000954*2);

  d_angle_x = angle_x*180;
  d_angle_y = angle_y*180;
  //accel calculation
  Roll = (float)atan2(Axyz[1],Axyz[2])*(180/3.1415);
  Pitch = (float)(-atan2(-Axyz[0], sqrt((Axyz[1] * Axyz[1]) + (Axyz[2] * Axyz[2]))) * (180 / 3.1415));
  //complimentary filter
  Roll_output = 0.98*(0.1*Roll_output+0.9*d_angle_x) + 0.02*Roll;
  Pitch_output = 0.98*(0.1*Roll_output+0.9*d_angle_y) + 0.02*Pitch;
  Roll = Roll_output - 3.32;
  Pitch = Pitch_output - 0.26;
}

