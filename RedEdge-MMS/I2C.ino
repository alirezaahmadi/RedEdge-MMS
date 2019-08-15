/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
bool I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
// Set register address
Wire.beginTransmission(Address);
Wire.write(Register);
Wire.endTransmission();
 
// Read Nbytes
Wire.requestFrom(Address, Nbytes); 
uint8_t index=0;
while (Wire.available())
Data[index++]=Wire.read();

if(sizeof(Data)!=0){return true;}
else {return false;}
}
 
 
// Write a byte (Data) in device (Address) at register (Register)
bool I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
// Set register address
Wire.beginTransmission(Address);
  if(Wire.write(Register))
    if(Wire.write(Data))
      if(Wire.endTransmission()){
        return true;
      }else{
        return false;
      }
}
