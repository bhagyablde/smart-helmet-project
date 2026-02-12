#include <MPU6050_tockn.h>
#include <Wire.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ThingSpeak.h>

const char* ssid = "abcd";   // Your Network SSID
const char* password = "12345678";       // Your Network Password
uint32_t tsLastReport = 0;  


WiFiClient client;

unsigned long myChannelNumber = 3142443; //Your Channel Number (Without Brackets)
const char * myWriteAPIKey = "EREZ1FGA3UDIRQ8Z"; //Your Write API Key

bool set = false;


#define REPORTING_PERIOD_MS     1000



 



MPU6050 mpu6050(Wire);

long timer = 0;
long timer1=0;
int initialZacc=0;

int relay =D5;
// int airbagrel=8;
int hel_sensor = D0;
//int das_sensor = 7;

int buz= D6;
int red=D3;  
int green = D4;

#include <TinyGPS++.h>
#include <SoftwareSerial.h>//Library used to create software serial port                                  
                  // library used to enable i2c port 
String textMessage;



  
int Number_of_SATS;       // Variable used to store number of satellite  
String latitude;         // variable to store latitude
String longitude;       // variable to store longitude                                 
TinyGPSPlus gps;         // create oject gps to class TinyGPSPLUS
SoftwareSerial serialGPS(D7,D8);  // create software serial "serial2" tx pin of gps to pin11 and rx to pin 10


 void onBeatDetected()  
 {  
  ;  
 } 

void setup() {
  Serial.begin(9600);
  Wire.begin();

      WiFi.begin(ssid, password);
  ThingSpeak.begin(client);
  delay(100);
  pinMode(relay,OUTPUT);
  //  pinMode(airbagrel,OUTPUT);
  pinMode(hel_sensor,INPUT);
pinMode(buz,OUTPUT);
pinMode(red,OUTPUT);
pinMode(green,OUTPUT);
  digitalWrite(relay,HIGH);
    digitalWrite(buz,LOW);
    analogWrite(red,120);
    analogWrite(green,0);
  // digitalWrite(airbagrel,HIGH);

 lcd.init();   // INITIALIZE THE LCD 
  lcd.backlight(); //  IT WILL TURN ON BACK LIGHT 
  lcd.clear();
  lcd.setCursor(3,0); ///  TO SET WRITING POSITION ON LCD ( COL,ROW)
  lcd.print("Calibrating");
 // ("  ")
  lcd.setCursor(3,1);
  lcd.print("Dont move ");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
  delay(5000);
  lcd.clear();

 serialGPS.begin(9600);//  serial2 is for communication with the computer with baud rate 9600
 

 }  

  
  
  


void loop() {
int hel_sensed= digitalRead(hel_sensor);
//int das_sensed= digitalRead(das_sensor);  
//Serial.print("helmate: ");
Serial.println(hel_sensed);
int alcohol=analogRead(A0);
// Serial.print("alcholo: ");
Serial.println(alcohol);
lcd.clear();
  lcd.setCursor(3,0); ///  TO SET WRITING POSITION ON LCD ( COL,ROW)
  lcd.print("Wear Helmet");
  delay(20);
  mpu6050.update();
 
  digitalWrite(buz,HIGH);

    analogWrite(red,120);
    analogWrite(green,0);
    delay(300);
      digitalWrite(buz,LOW);

    analogWrite(red,0);
    analogWrite(green,0);
 


 while(alcohol<1000 && hel_sensed==0){

  getGPS(); 

  
   digitalWrite(relay,LOW);
  // Serial.println("inside while");
      analogWrite(red,0);
    analogWrite(green,120);
    delay(200);
    analogWrite(red,0);
    analogWrite(green,0);
  digitalWrite(buz,LOW);
hel_sensed= digitalRead(hel_sensor);
  if(hel_sensed==1){
    digitalWrite(relay,HIGH);
  }
  lcd.clear();  
  lcd.setCursor(1,0); ///  TO SET WRITING POSITION ON LCD ( COL,ROW)
  lcd.print("Safty mode ON");
  delay(20);
  mpu6050.update();
   if(millis() - timer > 100){
    
    
   initialZacc=mpu6050.getAccY();
  
    timer = millis();
    
  }
  //Serial.println(initialZacc);

    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {

      ThingSpeak.setField( 1,latitude); //Update in ThingSpeak
  ThingSpeak.setField( 2,longitude); //Update in ThingSpeak
ThingSpeak.setField( 3,0); //Update in ThingSpeak

         ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);   // write all fields to the channel and reset stored 

    tsLastReport = millis();
    }
  if(abs(mpu6050.getAccY()- initialZacc )>0.97){
    getGPS(); 
set = true;
  digitalWrite(relay,HIGH);

  digitalWrite(buz,HIGH);
        analogWrite(red,120);
    analogWrite(green,0);
   lcd.clear();
    lcd.setCursor(2,0); ///  TO SET WRITING POSITION ON LCD ( COL,ROW)
  lcd.print("ACCIDENT");
 // Serial.println("accident detected");
  lcd.setCursor(2,1); ///  TO SET WRITING POSITION ON LCD ( COL,ROW)
  lcd.print("DETECTED");
while(set){
        ThingSpeak.setField( 1,latitude); //Update in ThingSpeak
  ThingSpeak.setField( 2,longitude); //Update in ThingSpeak
ThingSpeak.setField( 3,1); //Update in ThingSpeak

         ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);   // write all fields to the channel and reset stored 
}

delay(3000);
    digitalWrite(buz,LOW);
     getGPS();                 // Get gps data

    }

}
}


void getGPS()                                                 // Get Latest GPS coordinates
{

  while (serialGPS.available() > 0)             //check for gps data available in serial port
  {
    gps.encode(serialGPS.read());
  }                 // encode the gps data into latitude and longitude 

latitude =String( gps.location.lat(),6); //fe tch latitude and store 
longitude =String (gps.location.lng(),6); // fetch longitude and store 

}