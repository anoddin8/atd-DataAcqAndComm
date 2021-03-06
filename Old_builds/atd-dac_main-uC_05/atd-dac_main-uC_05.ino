// Libraries for SD card
#include "FS.h"
#include "SD.h"
#include <SPI.h>
// Additional libraries 
#include <string.h>
#include <string>
#include <iostream>
// BLE library
#include "BLEDevice.h"
// Library for Clock Module
#include<RTClib.h>
// Libraries to get time from NTP Server, and WiFi 
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

//Function Prototypes
void scanBLE(); //Scan for BLE
bool connectToServer(); //Connect to BLE
void getTimeStamp(); //RTC module Get Time and Date
void printDateTime();//Print Date And time
void createFileName(int fileNum); //Create unique filename
void logSDCard(int sensorValue); //log values to sd card
void writeFile(fs::FS &fs, const char * path, const char * message) ; //write file
void appendFile(fs::FS &fs, const char * path, const char * message); //append file

// Define CS pin for the SD card module
#define SD_CS 5
// Define Output Pins for LED
#define R_LED 25
#define G_LED 26
#define B_LED 27

//Define Pressure Sensor Analog Input Pin
#define PRESS_SENSE A0
//Define Session Sensor Input Pin
#define SESS_SENSE 2

// The remote service we wish to connect to
static BLEUUID serviceUUID("2c616054-b134-4af3-8828-10cf8b206752");
// The characteristics of the remote service we are interested in
static BLEUUID    charUUID_IMU1xyz("920eb6a5-4dd5-40e6-bd32-718fa583d8d6");
static BLEUUID    charUUID2_IMU2xyz("9d025389-2a3f-47f9-aab8-333ad88e67da");
static BLEUUID    charUUID3_diffAngle("a0208c9f-9763-409e-aa6c-6975fcc68cff");
//Flags for Bluetooth
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pRemoteCharacteristic2;
static BLERemoteCharacteristic* pRemoteCharacteristic3;
static BLEAdvertisedDevice* myDevice;

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

#define PRINT_SPEED 2000 // time between polling
static unsigned long lastPrint = 0; // Keep track of polling time

// Variables for SDcard File name and Days fo the week
char FileName[35];
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// WiFi Variables 
const char* ssid ="Thurdin";
const char* password="Rustle100";
//const char* ssid ="SM-G920W80460";
//const char* password="prts3279";
bool connWifi=false;
bool BluetoothOn=false;

//Pressure Variables
int16_t pressureSensor;
bool sessionSensor=true;

//BLE callback function
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) 
  {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
  }

class MyClientCallback : public BLEClientCallbacks 
{
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

// Scan for BLE servers and find the first one that advertises the service we are looking for.
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks 
{
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) 
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void setup() {
  Serial.begin(115200);

  pinMode(R_LED,OUTPUT);
  pinMode(G_LED,OUTPUT);
  pinMode(B_LED,OUTPUT);
  pinMode(SESS_SENSE,INPUT);
  digitalWrite(R_LED, LOW);
  digitalWrite(G_LED, LOW);
  digitalWrite(B_LED, LOW);

  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");
  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  delay(1000);
  WiFi.begin(ssid, password);
  delay(500);
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
    // GMT +1 = 3600, GMT +8 = 28800, GMT -1 = -3600, GMT 0 = 0
  timeClient.setTimeOffset(-18000); //Eastern Standard Time -18000
  
  // Initialize SD card
  SD.begin(SD_CS);  
  if(!SD.begin(SD_CS)) 
  {
    Serial.println("Card Mount Failed");
  }

  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) 
  {
    Serial.println("No SD card attached");
  }
    if (! rtc.begin()) 
    {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  
  if((WiFi.status() == WL_CONNECTED))
    getTimeStamp();
  else
    Serial.println("No time update without Internet Connection.");

  printDateTime();

  
}

void loop() 
{  
  //Scan for BLE until proper device is found 
  if(!doConnect && !BluetoothOn)
  {
    scanBLE(); 
  }
  
  //connect to BLE server if proepr device is found
  if (doConnect == true) 
  {
    Serial.println("Connecting to the BLE Server.");
    if (connectToServer()) 
    {
      Serial.println("We are now connected to the BLE Server.");
      BluetoothOn=true;
    } 
    else if(!BluetoothOn)
    {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
  doConnect = false;
  }
  
  //check session sensor
  sessionSensor=digitalRead(SESS_SENSE);

  //If session sensor on, wait 0.5s to insure session actually started
  if (sessionSensor && BluetoothOn)
    {
      delay(500);
      sessionSensor=digitalRead(SESS_SENSE);
    }
    
  // Run when therapy session has started
  if(sessionSensor && BluetoothOn)
  {
    int fileNum=0;
    bool uniqueFileName=false;

    //Create unique file name, and set header line on .csv file
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
    unsigned long lastPrint = 0;
    //When session sesnor is on log all data
    while(sessionSensor)
    {
      if((lastPrint + PRINT_SPEED) < millis())
      {
        // Insures session is still connected to BLE
        if (!connected && doScan) 
        {
          BLEDevice::getScan()->start(0);  
        }
        //Convert pressure sensor voltage to PSI
        pressureSensor = map(analogRead(PRESS_SENSE), 390, 3962, 0, 1200);
        if (pressureSensor < 0)
          {
            pressureSensor = 0;
          }
        Serial.println(double(pressureSensor)/100);
        
        //LogSDcard with Pressure data and extracted Bluetooth data
        logSDCard(pressureSensor);
        sessionSensor=digitalRead(SESS_SENSE);
        
        if (!sessionSensor)
          {
          delay(500);
          sessionSensor=digitalRead(SESS_SENSE);
          }
        lastPrint = millis(); // Update lastPrint time
      } 
    }
  }
}
//BLE Scan for Device
void scanBLE()
{
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}
//Form connection to BLE Device
bool connectToServer() 
{
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) 
    {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");
   // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID_IMU1xyz);
    if (pRemoteCharacteristic == nullptr) 
    {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID_IMU1xyz.toString().c_str());
      pClient->disconnect();
      return false;
    }
    pRemoteCharacteristic2 = pRemoteService->getCharacteristic(charUUID2_IMU2xyz);
    if (pRemoteCharacteristic2 == nullptr) 
    {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID2_IMU2xyz.toString().c_str());
      pClient->disconnect();
      return false;
    }
    pRemoteCharacteristic3 = pRemoteService->getCharacteristic(charUUID3_diffAngle);
    if (pRemoteCharacteristic3 == nullptr) 
    {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID3_diffAngle.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found all characteristics");

    // Read the value of the characteristic.
//    if(pRemoteCharacteristic3->canRead()) 
//    {
//      std::string value = pRemoteCharacteristic3->readValue();
//      Serial.print("The characteristic value was: ");
//      Serial.println(value.c_str());
//    }

    if(pRemoteCharacteristic3->canNotify())
      pRemoteCharacteristic3->registerForNotify(notifyCallback);

    connected = true;
    return connected;
}
//Get Time Stamp from internet and update RTC
void getTimeStamp() 
{
  String formattedDate;
  String YearStamp,MonthStamp, DayStamp;
  String timeStamp;

  while(!timeClient.update()) 
  {
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // Extract Date and time:
  formattedDate = timeClient.getFormattedDate();

  // Extract date
  YearStamp = formattedDate.substring(0, 4);
  MonthStamp = formattedDate.substring(5, 7);
  DayStamp = formattedDate.substring(8, 10);
  
  //Convert Date info to char arrays 
  char yr[YearStamp.length()+1];
  char mth[MonthStamp.length()+1];
  char dy[DayStamp.length()+1];
  YearStamp.toCharArray(yr,YearStamp.length());
  MonthStamp.toCharArray(mth,MonthStamp.length());
  DayStamp.toCharArray(dy,DayStamp.length());

  //Convert Date and Time Variables to Required type for RTC
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
//Print Date and time
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

// Write all sensor readings on the SD card
void logSDCard(int sensorValue) 
{
  // Set the characteristic's value to be the array of bytes that is actually a string.
      int IMU1x, IMU2x, IMU1y, IMU2y, IMU1z, IMU2z, diffAngle;
      std::string IMU1String=pRemoteCharacteristic->readValue();
      if (int(*IMU1String.c_str())==0)
        IMU1x=int(*(IMU1String.c_str()+1));
      else
        IMU1x=-1*(uint8_t(*(IMU1String.c_str()+1)));
      if (int(*IMU1String.c_str()+2)==0)
        IMU1y=int(*(IMU1String.c_str()+3));
      else
        IMU1y=-1*(uint8_t(*(IMU1String.c_str()+3)));
      if (int(*IMU1String.c_str()+4)==0)
        IMU1z=int(*(IMU1String.c_str()+5));
      else
        IMU1z=-1*(uint8_t(*(IMU1String.c_str()+5)));
           
      std::string IMU2String = pRemoteCharacteristic2->readValue(); //std::string
      if (int(*IMU2String.c_str())==0)
        IMU2x=int(*(IMU2String.c_str()+1));
      else
        IMU2x=-1*(uint8_t(*(IMU2String.c_str()+1)));
      if (int(*IMU2String.c_str()+2)==0)
        IMU2y=int(*(IMU2String.c_str()+3));
      else
        IMU2y=-1*(uint8_t(*(IMU2String.c_str()+3)));
      if (int(*IMU2String.c_str()+4)==0)
        IMU2z=int(*(IMU2String.c_str()+5));
      else
        IMU2z=-1*(uint8_t(*(IMU2String.c_str()+5)));

      std::string IMU3String = pRemoteCharacteristic3->readValue(); //std::string
      diffAngle=int(*IMU3String.c_str());
      Serial.println(diffAngle); 

  DateTime now = rtc.now();
  dataMessage = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) +"," + String(double(sensorValue)/100) + "," + String(diffAngle) + "," + 
                String(IMU1x) + "," + String(IMU1y) + "," + String(IMU1z) + "," + String(IMU2x) + "," + String(IMU2y) + "," + String(IMU2z) + "\r\n";
  Serial.println(dataMessage);
  appendFile(SD, FileName, dataMessage.c_str());
}

// Write to the SD card 
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

// Append data to the SD card 
void appendFile(fs::FS &fs, const char * path, const char * message) 
{
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
