void IMUSteup(void){
// Set accelerometers low pass filter at 5Hz
I2CwriteByte(MPU9250_ADDRESS,29,0x06);
// Set gyroscope low pass filter at 5Hz
I2CwriteByte(MPU9250_ADDRESS,26,0x06);
// Configure gyroscope range
I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
// Configure accelerometers range
I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
// Set by pass mode for the magnetometers
I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
// Request continuous magnetometer measurements in 16 bits
I2CwriteByte(MAG_ADDRESS,0x0A,0x16);  
}

void ReadAccelGyro(int16_t *IMU){
// ::: accelerometer and gyroscope :::
// Read accelerometer and gyroscope
uint8_t Buf[14];
I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
// Accelerometer
IMU[0]=-(Buf[0]<<8 | Buf[1]);
IMU[1]=-(Buf[2]<<8 | Buf[3]);
IMU[2]=Buf[4]<<8 | Buf[5];
// Gyroscope
IMU[3]=-(Buf[8]<<8 | Buf[9]);
IMU[4]=-(Buf[10]<<8 | Buf[11]);
IMU[5]=Buf[12]<<8 | Buf[13];
}

