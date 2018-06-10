/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RedEdge MMS-Box                                        %
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AhmadiAlireza.webs.com                                 %
% Date: 5.2018                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "SSD1306AsciiAvrI2c.h"   //OLED LCD   library
//#include "TimerOne.h"             //Hardware Timer One library
#include <VL53L0X.h>              //ToF sensor library
#include <SPI.h>                  //SPI interface linrary
#include <SD.h>                   //SD card interface library
#include <Wire.h>

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00 
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18
 
#define ACC_FULL_SCALE_2_G 0x00 
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

// Define proper RST_PIN if required.
#define RST_PIN -1

#define I2C_ADDRESS 0x3C          
//********** Modules Initial objects ***************
SSD1306AsciiAvrI2c oled;
VL53L0X sensor;
File myFile;
//************* Joy stick Pins *********************
#define joyPin1   0                 // slider variable connecetd to analog pin 0
#define joyPin2   1                 // slider variable connecetd to analog pin 1
#define buttonPin A2                // Joy button

#define MMC_CS    4

#define ShotTrig 10

#define DutyCycle 512

//#define Debbug_mode 
//#define Setup_mode
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
void ReadAccelGyro(int16_t *IMU);
void IMUSteup(void);


struct _labels{
  uint16_t id;
  char* Name;
};

bool button_press(byte Btn_Num, unsigned short Debaouce=100)
{
  //Debaouce=100;
  if (digitalRead(Btn_Num) == 0)
  {
    long elapsed = (unsigned short)millis();
    for (;;)
    {
      if ((unsigned short)Debaouce < (unsigned short)(millis() - elapsed))
      {
        elapsed=0;
        return true;
      }
    }
  }
}

bool Joy_press(int Joy_axis,int index, uint16_t Threshold_low,uint16_t Threshold_high, unsigned short Debaouce=100)
{
  unsigned short elapsed = 0;
  if (analogRead(Joy_axis) >= Threshold_high)
  {
    elapsed = (unsigned short)millis();
    for (;;)
    {
      if ((unsigned short)Debaouce < (unsigned short)(millis() - elapsed))
      {
        index++;
        elapsed=0;
        return true;
        break;
      }
    }
  }else if(analogRead(Joy_axis) <= Threshold_low){
    elapsed = (unsigned short)millis();
    for (;;)
    {
      if ((unsigned short)Debaouce < (unsigned short)(millis() - elapsed))
      {
        index--;
        elapsed=0;
        return true;
        break;
      }
    }
  }
}

String readLine() {
  String received = "";
  char ch;
  while(myFile.available()){
    ch = myFile.read();
    if(ch == '.') {
      return String(received);
    } else {
      received+=ch;
    }
  }
  return "";
}

_labels PlantCrop[30];
uint8_t lable_index=0;

uint8_t Joy_read(uint16_t UpDwon_threshold, uint16_t LeftRight_threshold){
  //digitalRead(joyPin1);
}

void writeString(String stringData) { // Used to serially push out a String with Serial.write()
  for (int i = 0; i < stringData.length(); i++)
  {
    Serial.write(stringData[i]);   // Push each char 1 by 1 on each loop pass
  }
}

void setup() {

  pinMode(ShotTrig, OUTPUT);digitalWrite(ShotTrig,LOW);
  pinMode(buttonPin, INPUT);
  pinMode(SS, OUTPUT);

  //Timer1.initialize(500000);        // initialize timer1, and set a 1/2 second period
  //Timer1.pwm(ShotPWM, 512);         // setup pwm on pin 9, 50% duty cycle

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Serial initialization Done!");

//******************* MMC-card Initialization ***************
#ifdef Setup_mode
#elif Debbug_mode
#else
  Serial.print("Initializing SD card...");
  if (!SD.begin(MMC_CS)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  myFile = SD.open("lables.txt");
  if (myFile) {
    int cnt=0;
    while (myFile.available()) {
      if(cnt%2 == 0){
        PlantCrop[cnt].id = myFile.parseInt();
      }else{
        PlantCrop[cnt].Name = myFile.read();
      }
      cnt++;
      //Serial.println(cnt);
      if(cnt ==25)break;
    }
     myFile.close();
     myFile = SD.open("lables.txt");
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
    Serial.println("lables closed.");
    for(int i=0;i<25;i++){
      Serial.print(PlantCrop[i].id);
      Serial.print(":  ");
      Serial.print(PlantCrop[i].Name);
      Serial.println();
    }
  }
#endif
//******************* IMU Initialization ***************
#ifdef Setup_mode
#elif Debbug_mode
  
#else
  IMUSteup();
#endif  

////******************* ToF Initialization ***************
#ifdef Setup_mode
#elif Debbug_mode
#else
  sensor.init();
  sensor.setTimeout(1);
  sensor.startContinuous();
#endif 

////******************* LCD Initialization ***************
#ifdef Setup_mode
#elif Debbug_mode
#else
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);      //Initial font style
  oled.clear();
  oled.set2X();                   // set to bigger font
  oled.println("RedEdgeMMS");
  oled.set1X();
  oled.print("        6/2108");
  delay(1000);
  oled.clear();
#endif

PlantCrop[0].id=1;
PlantCrop[0].Name = "Plant-A";
PlantCrop[1].id=2;
PlantCrop[1].Name = "Plant-B";
PlantCrop[2].id=3;
PlantCrop[2].Name = "Plant-C";
PlantCrop[3].id=4;
PlantCrop[3].Name = "Plant-D";
PlantCrop[4].id=5;
PlantCrop[4].Name = "Plant-E";
}

//------------------------------------------------------------------------------
void loop() {
#ifdef Setup_mode
  // in case of Setup mode -- loop
#elif Debbug_mode
sd_raw  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);
  Serial.print(" \t");
  Serial.print(value1);
  Serial.print(" \t");
  Serial.print(value1);
  Serial.print(" \t");
  Serial.println(button);
#else
  oled.clear();
  oled.set1X();
  oled.print(sensor.readRangeContinuousMillimeters());
  oled.print("     ");
  oled.println(" 00");
  // second row
  oled.set2X();
  oled.println(PlantCrop[lable_index].Name);
  oled.set1X();
  oled.print(lable_index);//oled.print("  ");oled.print(value2);oled.print("  ");oled.println(button);
  Joy_press(joyPin1,lable_index, 50,600);
  Joy_press(joyPin2,lable_index, 50,600);
  if(button_press(buttonPin)){
    //Shot - write to sd
  }
  
  //delay(100);
#endif
}


