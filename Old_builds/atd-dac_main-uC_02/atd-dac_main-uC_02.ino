// Libraries for SD card
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <string.h>
#include <string>
#include <iostream>

// Library for Clock Module
#include<RTClib.h>


// Libraries to get time from NTP Server
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Replace with your network credentials
//const char* ssid     = "Welcome to Humber";//"SM-G920W80460";
//const char* password = "";

// Define CS pin for the SD card module
#define SD_CS 5
// Define Pins for LED
#define R_LED 25
#define G_LED 26
#define B_LED 27
//Define Pressure Sensor Pin
#define PRESS_SENSE A0
//Define Session Sensor
#define SESS_SENSE 2


// Save reading number on RTC memory
RTC_DATA_ATTR int readingID = 0;

//Clock Module declaration
RTC_DS3231 rtc;

String dataMessage;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Define CS pin for the SD card module
#define SD_CS 5

// Variables to save date and time
String formattedDate;
String YearStamp,MonthStamp, DayStamp;
String timeStamp;
char FileName[35];
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


// WiFi Variables 
const char* ssid ="Thurdin";
const char* password="Rustle100";
bool connWifi=false;

//Pressure Variables
int pressureSensor;
int sessionSensor=0;



void setup() {
  Serial.begin(115200);

   pinMode(R_LED,OUTPUT);
   pinMode(G_LED,OUTPUT);
   pinMode(B_LED,OUTPUT);
   pinMode(SESS_SENSE,INPUT);
   digitalWrite(R_LED, LOW);
   digitalWrite(G_LED, LOW);
   digitalWrite(B_LED, LOW);
   
  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  for (int i =0;i<50 && (WiFi.status() != WL_CONNECTED);i++)
  {
    delay(250);
    digitalWrite(R_LED, HIGH);
    delay(250);
    digitalWrite(R_LED, LOW);
    Serial.print(".");
  }
  Serial.println("");
  if(WiFi.status() == WL_CONNECTED)
    {
      Serial.println("WiFi connected.");
      digitalWrite(G_LED, HIGH);
    }
  else
    {
      Serial.println("No connection Found");
      digitalWrite(R_LED, HIGH);
    }

   // Initialize a NTPClient to get time
  timeClient.begin();
    // Set offset time in seconds to adjust for your timezone, for example:
    // GMT +1 = 3600
    // GMT +8 = 28800
    // GMT -1 = -3600
    // GMT 0 = 0
  timeClient.setTimeOffset(-18000); //Eastern Standard Time -18000

  // Initialize SD card
  SD.begin(SD_CS);  
  if(!SD.begin(SD_CS)) 
  {
    Serial.println("Card Mount Failed");
  }

  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
  }
    if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if((WiFi.status() == WL_CONNECTED))
    getTimeStamp();
  else
    Serial.println("No time update without Internet Connection.");
  
   
  printDateTime();

  
}

void loop() {
  sessionSensor=digitalRead(SESS_SENSE);
  if (sessionSensor)
    {
      delay(500);
      sessionSensor=digitalRead(SESS_SENSE);
    }
  
  if(sessionSensor){
    int fileNum=0;
    bool uniqueFileName=false;
    while(!uniqueFileName)
    {
      createFileName(fileNum);
      File file = SD.open(FileName);
      if(!file) {
        Serial.println("Creating file...");
        writeFile(SD, FileName, "Time, Pressure, Diff Angle, IMU1_X, IMU1_Y, IMU1_Z, IMU2_X, IMU2_Y, IMU2_Z  \r\n");
        uniqueFileName=true;
      }
      else {
        fileNum++;  
      }
      file.close();
    }
    
    while(sessionSensor)
    {
      pressureSensor = map(analogRead(PRESS_SENSE), 390, 3962, 0, 1200);
        if (pressureSensor < 0)
        {
          pressureSensor = 0;
        }
      Serial.println(double(pressureSensor)/100);
      logSDCard(pressureSensor);
      delay(1500);
      sessionSensor=digitalRead(SESS_SENSE);
        if (!sessionSensor)
          {
          delay(500);
          sessionSensor=digitalRead(SESS_SENSE);
          }
    }
  }
}
//Get Time Stamp from internet and update RTC
void getTimeStamp() 
{
  while(!timeClient.update()) 
  {
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = timeClient.getFormattedDate();

  // Extract date
  YearStamp = formattedDate.substring(0, 4);
  MonthStamp = formattedDate.substring(5, 7);
  DayStamp = formattedDate.substring(8, 10);

  char yr[YearStamp.length()+1];
  char mth[MonthStamp.length()+1];
  char dy[DayStamp.length()+1];

  YearStamp.toCharArray(yr,YearStamp.length());
  MonthStamp.toCharArray(mth,MonthStamp.length());
  DayStamp.toCharArray(dy,DayStamp.length());
  //char yr[4],mth[2],dy[2],hr[2],mn[2],sc[2];
  uint16_t Iyr;
  uint8_t Imth,Idy,Ihr,Imn,Isc;
  Iyr=(uint16_t)YearStamp.toInt();
  Imth=(uint8_t)MonthStamp.toInt();
  Idy=(uint8_t)DayStamp.toInt();
  Ihr=(uint8_t)timeClient.getHours();
  Imn=(uint8_t)timeClient.getMinutes();
  Isc=(uint8_t)timeClient.getSeconds();
  rtc.adjust(DateTime(Iyr, Imth, Idy, Ihr, Imn, Isc));
  
}
//Print Dare and time
void printDateTime()
{
    DateTime now = rtc.now();
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
}
//Create New File Name
void createFileName(int fileNum)
{
  DateTime now = rtc.now();
  String FileNameStr="/";
  FileNameStr += now.year();
  FileNameStr += "-";
  FileNameStr += now.month();
  FileNameStr += "-";
  FileNameStr += now.day();
  FileNameStr += "_ATD_DATA_";
  if(fileNum>=100)
    FileNameStr += fileNum;
  else if (fileNum>=10)
  {
    FileNameStr += "0";
    FileNameStr += fileNum;
  }
  else
  {
    FileNameStr += "0";
    FileNameStr += "0";
    FileNameStr += fileNum;
  }
  FileNameStr += ".csv";
  
  FileNameStr.toCharArray(FileName,35);
  //Serial.println(FileName);
    
}

// Write the sensor readings on the SD card
void logSDCard(int sensorValue) {
  DateTime now = rtc.now();
  dataMessage = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) +"," + String(double(pressureSensor)/100) + "," + "65" + "," + 
                "50" + "," + "90" + "," + "30" + "," + "50" + "," + "25" + "," + "30" + "\r\n";

  appendFile(SD, FileName, dataMessage.c_str());
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) 
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  //Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    //Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}
