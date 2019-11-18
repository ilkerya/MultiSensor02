// new version with setrial comm for WIFI Module
#include <Adafruit_SSD1306.h>
#include "SparkFun_Si7021_Breakout_Library.h"
#include <MutichannelGasSensor.h>
#include <MICS-VZ-89TE.h>
#include <Wire.h>
//#include "utility/twi.h"

#define TCAADDR 0x71
#define CHANNEL_VOC 2
#define CHANNEL_TEMP 7
#define CHANNEL_GAS 6


unsigned int PreheatTimer = 0;
unsigned int IntTimer = 0;
bool Preheat = false;
unsigned int Timeout;
byte rx_byte = 0;        // stores received byte    
byte SerialCounter = 0; 
int buttonCounter = 0;





#define SensorAnalogPin A2  //this pin read the analog voltage from the HCHO sensor
#define VREF  5.0   //voltage on AREF pin


#include "C:\Projects\MKR1000_M2X\Common.h"
#include "C:\Projects\MKR1000_M2X\SensorsMega.h"
#include "C:\Projects\MKR1000_M2X\DispPrintMega.h"



void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);  // start serial for output
    delay(10);

    DisplayInit();

    Wire.begin();
     delay(20);
     
    tcaselect(CHANNEL_GAS);
      Serial.println("power on!");
    gas.begin(0x04);//the default I2C address of the slave is 0x04
    gas.powerOn();
    Serial.print("Firmware Version = ");
    Serial.println(gas.getVersion());

      tcaselect(CHANNEL_VOC);
    VOCSensor.begin();
    VOCSensor.getVersion();
  //  Serial.println(data[0]);
 //  Serial.println(data[1]);
  //  Serial.println(data[2]);

  tcaselect(CHANNEL_TEMP);
  Si072_Sensor.begin();
    
    Serial2.begin(115200);  // start serial for output
    Serial3.begin(115200);  // start serial for output  

             // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 34286;            // preload timer 65536-16MHz/256/2Hz
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts(); 
}




ISR(TIMER1_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
    TCNT1 = 34286;            // preload timer
    if(IntTimer == 0)Loop.Task1 = 1;
    if(IntTimer == 2)Loop.Task2 = 1;
    if(IntTimer == 4)Loop.Task3 = 1;
    if(IntTimer == 6)Loop.Task4 = 1; 
    if(IntTimer == 8)Loop.Task5 = 1;
    if(IntTimer == 10)Loop.Task6 = 1;
    IntTimer++;
    if(IntTimer > 11)IntTimer = 0; 
  // digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
}



void EndReceive(){
    SerialCounter = 0;
    Serial2.end();
    Serial2.begin(115200);
    rx_byte = 0;
    Serial.println(' ');
  
}

void PrintSerialLoop(){
  Serial.print(SerialCounter);Serial.print('.'); Serial.println(rx_byte);
}

void loop() {
      // http://wiki.seeed.cc/Grove-Multichannel_Gas_Sensor/
      // https://www.dfrobot.com/product-1574.html
      // https://www.mouser.se/datasheet/2/18/MiCS-VZ-89TE-V1.0-1100483.pdf




     if(Loop.Task1){
        Loop.Task1 = 0;
        TempSensorMeasure();
        TempSensorPrint();  
    //    Serial2.print("AAAB12345678901234567890123456A0A1");   


       //      unsigned int *p;
      //      p = (unsigned int*)&Values;

        
        
      
     }
     if(Loop.Task2){
        Loop.Task2 = 0;
        GasSensorMeasure();
        GasSensorPrint();  
  
     }
     if(Loop.Task3){
        Loop.Task3 = 0;


     }
      if(Loop.Task4){
        Loop.Task4 = 0;
        VOCSensorMeasure();
        VOCSensorPrint();        
        Serial.println(" ");   
 
     }    
     if(Loop.Task5){
        Loop.Task5 = 0;
        displayValues();     
     }
     if(Loop.Task6){
        Loop.Task6 = 0;

      for(int i = 0; i<6; i++){
        if(ReceiveArray[i] = 0); //clean receive array
      }
      ReceiveCounter = 0;
     while (Serial2.available() > 0) {  
         ReceiveArray[ReceiveCounter] = Serial2.read();
         ReceiveCounter++;
        if(ReceiveCounter >= 6)break; // do not overflow the array        
       // PrintSerialLoop(); 
        
      }
     while (Serial2.available() > 0) {  
        ReceiveCounter = Serial2.read(); // flush if remaining     
     }
     
      EnableDataSend = ON;
      Serial.print("ReceiveArray:"); 
      for(int i = 0; i<6; i++){
        Serial.print(ReceiveArray[i]);Serial.print('.');
        if(ReceiveArray[i] != '5')EnableDataSend = OFF;
      }
       Serial.print("EnableDataSend:"); Serial.println(EnableDataSend);     
    if ( EnableDataSend == ON){
      EnableDataSend = OFF;
      byte ArraySize = sizeof(Values) + 7; // preamble 4 + size 1 + crc 2

        SendArray[0] = 'A';
        SendArray[1] = 'A';     
        SendArray[2] = 'A';
        SendArray[3] = 'A';           
        SendArray[4] = ArraySize;    
        byte *p = (byte*)&Values;
       for (int i= 5; i < ArraySize-2; i++){        
         SendArray[i]= *p;
         p++;              
      }  
      unsigned short CRCvalue = calcrc((char*)SendArray, ArraySize-2);              
     //  byte Preamble[5] = {'A','A','A','A',3};   
        SendArray[ArraySize-2] = (byte)CRCvalue;     // crc      
        SendArray[ArraySize-1] = (byte)(CRCvalue>>8);     // crc

        Serial.print("CRC [5]");Serial.println(SendArray[ArraySize-2]);
        Serial.print("CRC [6]");Serial.println(SendArray[ArraySize-1]);
        Serial.print("CRCvalue");Serial.println(CRCvalue);
         Serial2.write((byte*)SendArray, ArraySize);
      /*
      Serial.println("Begin to calibrate...");
      gas.doCalibrate();
      Serial.println("Calibration ok"); 
      gas.display_eeprom();  
      */ 
          }
          Timeout++;
          PreheatTimer++;
          if(PreheatTimer  == 480 && (Preheat == false) ){
            //displayValues(); 
            Preheat = true; 
     }           
   }      
}
