// Libraries for SD card
//#include "FS.h"
#include "SD.h"
#include <SPI.h>

//Additional libraries
#include <string>
#include <iostream>

//BLE library
#include "BLEDevice.h"

// Library for Clock Module
#include<RTClib.h>


// Libraries to get time from NTP Server, and WiFi 
#include <WiFi.h>
#include <NTPClient.h>
//#include <WiFiUdp.h>


//Function Prototypes
bool connectToServer(); //BLE
void getTimeStamp(); //RTC module
void printDateTime();//Print Date And time
void createFileName(int fileNum); //Create unique filename
void logSDCard(int sensorValue); //log values to sd card
void writeFile(fs::FS &fs, const char * path, const char * message) ; //write file
void appendFile(fs::FS &fs, const char * path, const char * message); //append file


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

// The remote service we wish to connect to.
static BLEUUID serviceUUID("2c616054-b134-4af3-8828-10cf8b206752");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("920eb6a5-4dd5-40e6-bd32-718fa583d8d6");
static BLEUUID    charUUID2("9d025389-2a3f-47f9-aab8-333ad88e67da");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pRemoteCharacteristic2;
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

// Variables to save date and time
//String formattedDate;
//String YearStamp,MonthStamp, DayStamp;
//String timeStamp;
char FileName[35];
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


// WiFi Variables 
//const char* ssid ="Thurdin";
//const char* password="Rustle100";
const char* ssid ="SM-G920W80460";
const char* password="prts3279";
bool connWifi=false;

//Pressure Variables
int16_t pressureSensor;
bool sessionSensor=true;

//BLE callback fumction
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



/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
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

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
 // End of setup.

  
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
  
  if((WiFi.status() == WL_CONNECTED))
    getTimeStamp();
  else
    Serial.println("No time update without Internet Connection.");

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
 
  printDateTime();

  
}

void loop() 
{
  //connect to BLE server
  if (doConnect == true) 
  {
    if (connectToServer()) 
    {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }
  //check session sensor
  sessionSensor=digitalRead(SESS_SENSE);
  if (sessionSensor)
    {
      delay(500);
      sessionSensor=digitalRead(SESS_SENSE);
    }
    
  // Run when therapy session has started
  if(sessionSensor)
  {
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
        if (connected) 
        {
          //String newValue = "Time since boot: " + String(millis()/1000);
          //Serial.println("Setting new characteristic value to \"" + newValue + "\"");
          
      
        }else if(doScan)
        {
          BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
        }
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
//BLE Function
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
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) 
    {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    pRemoteCharacteristic2 = pRemoteService->getCharacteristic(charUUID2);
    if (pRemoteCharacteristic == nullptr) 
    {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) 
    {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

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
void logSDCard(int sensorValue) 
{
  // Set the characteristic's value to be the array of bytes that is actually a string.
      int IMU1, IMU2;
      std::string IMU1String=pRemoteCharacteristic->readValue();
      if (int(*IMU1String.c_str())==0)
        IMU1=int(*(IMU1String.c_str()+1));
      else
        IMU1=-1*(uint8_t(*(IMU1String.c_str()+1)));
      
      
       
      std::string IMU2String = pRemoteCharacteristic2->readValue(); //std::string
      if (int(*IMU2String.c_str())==0)
        IMU2=int(*(IMU2String.c_str()+1));
      else
        IMU2=-1*(uint8_t(*(IMU2String.c_str()+1)));

     

      Serial.println(IMU1);
      Serial.println(IMU2);
  DateTime now = rtc.now();
  dataMessage = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) +"," + String(double(sensorValue)/100) + "," + "65" + "," + 
                "150"/*IMU1x.c_str()*/ + "," + "90" + "," + "30" + "," + "150"/*IMU2x.c_str()*/ + "," + "25" + "," + "30" + "\r\n";

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
