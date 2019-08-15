/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
  asm volatile ("  jmp 0");  
}  
void Error_status_LCD(String Message){
  oled.clear();
  oled.set2X();                   // set to bigger font
  oled.println("   Error");
  oled.set1X();
  oled.println(Message);
  oled.print("Skip");
  oled.println("            Reset");
  delay(50);
}

bool WriteLineToSSD(File& myFile, uint16_t ImageNumber,uint16_t id,String Name,uint16_t distance,float roll,float pitch){
  myFile = SD.open("log.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    //myFile.println("testing 1, 2, 3.");
    myFile.print(ImageNumber);myFile.print("\t");
    myFile.print(id);myFile.print("\t");
    myFile.print(distance);myFile.print("\t");
    myFile.print(roll);myFile.print("\t");
    myFile.print(pitch);myFile.print("\t");
    myFile.println(Name);
    // close the file:
    myFile.flush();
    myFile.close();
    Serial.print(ImageNumber);
    Serial.print("\t");
    Serial.print(id);
    Serial.print("\t");
    Serial.print(distance);
    Serial.print("\t");
    Serial.print(roll);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.println(Name);
  } else {
    // if the file didn't open, print an error:
   Serial.println("Write - error opening log.txt");
    
  }

//    // re-open the file for reading:
//  myFile = SD.open("log.txt");
//  if (myFile) {
//    Serial.println("log.txt:");
//
//    // read from the file until there's nothing else in it:
//    while (myFile.available()) {
//      Serial.write(myFile.read());
//    }
//    // close the file:
//    myFile.flush();
//    myFile.close();
//  } else {
//    // if the file didn't open, print an error:
//    Serial.println("Read - error opening log.txt");
//  }
  return true;
}

bool button_press(byte Btn_Num,uint16_t& ImageNumber, unsigned short Debaouce=100){
  //Debaouce=100;
  if (digitalRead(Btn_Num) == 1)
  {
    long elapsed = (unsigned short)millis();
    for (;;)
    {
      if ((unsigned short)Debaouce < (unsigned short)(millis() - elapsed))
      {
        elapsed=0;
        ImageNumber++;
        return true;
      }
    }
  }
  else{
    return false;
  }
}

bool Joy_press(int Joy_axis,uint8_t LableCNT,int8_t& index, uint16_t Threshold_low,uint16_t Threshold_high, unsigned short Debaouce=150){
  bool status = false;
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
        status = true;
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
        status = true;
        break;
      }
    }
  }
  if(index < 0 ) index = LableCNT-1;
  else if(index > LableCNT-1 ) index = 0;
  return status;
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

void writeString(String stringData) { // Used to serially push out a String with Serial.write()
  for (int i = 0; i < stringData.length(); i++)
  {
    Serial.write(stringData[i]);   // Push each char 1 by 1 on each loop pass
  }
}
