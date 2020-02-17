//<pre>#include &lt;Wire.h&gt;
// Libraries for SD card
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <string.h>

// Library for Clock Module
#include<RTClib.h>
#include<DS3231.h>

// Libraries to get time from NTP Server
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Replace with your network credentials
//const char* ssid     = "Welcome to Humber";//"SM-G920W80460";
//const char* password = "";

// Define CS pin for the SD card module
#define SD_CS 5

// Save reading number on RTC memory
RTC_DATA_ATTR int readingID = 0;

//Clock Module declaration
RTC_DS1307 rtc;

String dataMessage;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;
char FileName[25];
bool usbConn=false;
char ssid[25];

void setup() {
  Serial.begin(115200);
  
}

void loop() {
  // check if usb connected
if (Serial.available()) {
        usbConn=true;
        Serial.println("Connected to USB");
} else {
        usbConn=false;
}
if(usbConn && ){
    Serial.println("Enter Wifi SSID");
    Serial.read(ssid);
    
}


}
