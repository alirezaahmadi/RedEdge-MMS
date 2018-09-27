/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RedEdge MMS-Box                                        %
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AhmadiAlireza.webs.com                                 %
% Date: 5.2018                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include <Wire.h>
#include <VL53L0X.h>              //ToF sensor library
#include "SSD1306AsciiAvrI2c.h"   //OLED LCD   library
//#include "TimerOne.h"             //Hardware Timer One library
#include <SPI.h>                  //SPI interface linrary
#include <SD.h>                   //SD card interface library

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define LED 9

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
VL53L0X sensor;
SSD1306AsciiAvrI2c oled;
File myFile;
//************* Joy stick Pins *********************
#define joyPin1   A0                 // slider variable connecetd to analog pin 0
#define joyPin2   A1                 // slider variable connecetd to analog pin 1
#define buttonPin A2                // Joy button

#define MMC_CS    2

#define ShotTrig  6

#define DutyCycle 512

#define LaserPoint A3
#define LaserDealy 1000

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY

//#define Debbug_mode 
void software_Reset();
bool I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
bool I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
void getRollPitchYaw(float& roll,float& pitch);
bool IMUSteup(void);
bool WriteLineToSSD(File& myFile, uint16_t ImageNumber,uint8_t id,String Name,uint16_t distance,float roll,float pitch);
bool button_press(byte Btn_Num,uint16_t& ImageNumber, unsigned short Debaouce=100);
bool Joy_press(int Joy_axis,uint8_t LableCNT,int8_t& index, uint16_t Threshold_low,uint16_t Threshold_high, unsigned short Debaouce=150);
void writeString(String stringData);
void Error_status_LCD(String Message);

struct _labels{
  uint16_t id;
  char Name[10];
};

uint16_t ImageNumber = 0;
uint16_t Error_count =0;
_labels PlantCrop[30];
int8_t lable_index=0;
uint8_t PC_cnt=0;
int8_t tmp_index=1;


void setup() {
  Wire.begin();
  pinMode(ShotTrig, OUTPUT);digitalWrite(ShotTrig,LOW);
  pinMode(LaserPoint, OUTPUT);digitalWrite(LaserPoint,HIGH);
  pinMode(buttonPin, INPUT);
  pinMode(SS, OUTPUT);
  Serial.begin(115200);
#ifdef Debbug_mode
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif
delay(100);
////******************* LCD Initialization ***************
#ifdef Debbug_mode
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);      //Initial font style
  oled.clear();
  oled.set2X();                   // set to bigger font
  oled.println("RedEdgeMMS");
  oled.set1X();
  oled.print("        6/2108");
  delay(1000);
  oled.clear();
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
  delay(100);
////******************* ToF Initialization ***************
#ifdef Debbug_mode
#else
  if (!sensor.init()) {
    Error_status_LCD("Cant access to ToF!!");
    while (1){
       Joy_press(joyPin2, 3, tmp_index, 100, 500, 250);
       if(tmp_index==0){
        break;
       }else{
        software_Reset();
       }
    }
  }
  sensor.setTimeout(100);
  sensor.startContinuous(10);

  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif
  
  #if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensor.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    sensor.setMeasurementTimingBudget(200000);
  #endif
#endif
delay(100);
//******************* MMC-card Initialization ***************
#ifdef Debbug_mode
  if (!SD.begin(MMC_CS)) {
    Serial.println("SD initialization failed!");
    Error_status_LCD("Cant access to SD!!");
    while (1){
       Joy_press(joyPin2, 3, tmp_index, 100, 500, 250);
       if(tmp_index==0){
        break;
       }else if(tmp_index==2){
        software_Reset();
       }
    }
  }
  Serial.println("SD initialization done.");
//**********************************************************
  myFile = SD.open("lables.txt");
  if(myFile){
    String inData;
    while (myFile.available()) {
        char in_char = myFile.read();
        inData += in_char; 
        if (in_char == '\n')
            {
                Serial.print("Arduino Received: ");
                Serial.println(inData);
    //
    // this is where I need to split inData into the five integer values
    // and then do something with each individually
    // 
                inData = ""; // Clear received buffer      
    
            }
    }
    myFile.flush();
    myFile.close();
    Serial.println("lables closed.");
  }else{
    Serial.println("Cant Open labes.txt !!!");
  }
//******************************************
#else
  if (!SD.begin(MMC_CS)) {
    Error_status_LCD("Cant access to SD!!");
    while (1){
       Joy_press(joyPin2, 3, tmp_index, 100, 500, 250);
       if(tmp_index==0){
        break;
       }else if(tmp_index==2){
        software_Reset();
       }
    }
  }
  myFile = SD.open("lables.txt");
  if(myFile){
    uint8_t cnt=0;
    String inData;
    while (myFile.available()) {
      char in_char = myFile.read();
      inData += in_char; 
      if (in_char == '\n')
        {
          if(isDigit(inData.charAt(0))){
            PlantCrop[PC_cnt].id = inData.toInt();
          }else if(isAlphaNumeric(inData.charAt(0))){
            inData.toCharArray(PlantCrop[PC_cnt].Name,inData.length()); 
            //Serial.println(PlantCrop[PC_cnt].Name);
            //Serial.println(inData);              
//            if(str(PlantCrop[PC_cnt].Name) != inData){
//              inData.toCharArray(PlantCrop[PC_cnt].Name,inData.length()); 
//              Serial.println(PlantCrop[PC_cnt].Name);
//              Serial.println(inData);
//            }
            PC_cnt++;
          }
          cnt++; 
          inData = ""; // Clear received buffer    
        }
    }
    myFile.flush();
    myFile.close();
    cnt=0;
  }
#endif
delay(100);  
//******************* IMU Initialization ***************
#ifdef Debbug_mode
  if (!IMUSteup()) {
    Serial.println("SD initialization failed!");
    Error_status_LCD("Cant access to IMU!!");
    while (1){
       Joy_press(joyPin2, 3, tmp_index, 100, 500, 250);
       if(tmp_index==0){
        break;
       }else if(tmp_index==2){
        software_Reset();
       }
    }
  }
#else
  if (!IMUSteup()) {
    Serial.println("SD initialization failed!");
    Error_status_LCD("Cant access to IMU!!");
    while (1){
       Joy_press(joyPin2, 3, tmp_index, 100, 500, 250);
       if(tmp_index==0){
        break;
       }else if(tmp_index==2){
        software_Reset();
       }
    }
  }
#endif 
delay(100);
}
//******************* Main Loop ***************
void loop() {
digitalWrite(LED,HIGH);
uint16_t distance = 0;
uint16_t distance_tmp = 0;
float roll = 0.0;
float pitch = 0.0;
  distance_tmp = sensor.readRangeContinuousMillimeters();
  if(distance_tmp < 2000 && distance_tmp!= 0)distance = distance_tmp;
  getRollPitchYaw(roll,pitch);
  oled.clear();
  oled.set1X();
  oled.print("Height:");
  oled.print(distance/10);
  oled.print(" ");
  oled.print("PicNum:");
  oled.println(ImageNumber);
  if(button_press(buttonPin,ImageNumber)){
    if(WriteLineToSSD(myFile,ImageNumber,PlantCrop[lable_index].id,PlantCrop[lable_index].Name,distance,roll,pitch)){
      digitalWrite(LaserPoint,LOW);
      delay(50);
      digitalWrite(ShotTrig,HIGH);
      delay(LaserDealy);
    }else{
      oled.clear();
      while(1){
        digitalWrite(ShotTrig,LOW);
        oled.set2X();
        oled.print("Error SD card");
        if(button_press(buttonPin,Error_count))break;
      }
    }
    
  }
  digitalWrite(ShotTrig,LOW);
  digitalWrite(LaserPoint,HIGH);
  //second row
  oled.set2X();
  oled.println(PlantCrop[lable_index].Name);
  //Serial.println(PlantCrop[lable_index].Name);
  oled.set1X();
  oled.print(PlantCrop[lable_index].id);
  oled.print("  ");
  oled.print(roll);
  oled.print("  ");
  oled.print(pitch);
  //Serial.print(analogRead(joyPin1));
  //Serial.print("\t");
  //Serial.println(analogRead(joyPin2));
  //Joy_press(joyPin1, PC_cnt, lable_index, 100, 900, 250);
  Joy_press(joyPin2, PC_cnt, lable_index, 100, 900, 250);
  digitalWrite(LED,LOW);
  delay(10);
}


