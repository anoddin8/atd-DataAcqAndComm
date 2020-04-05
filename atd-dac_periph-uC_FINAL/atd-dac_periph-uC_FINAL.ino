/*****************************************************************

*****************************************************************/
//Libraries for IMUs
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
//Math functions for IMU calculations
#define _USE_MATH_DEFINES
#include <math.h>
//BLE Required Libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Creates objects for both IMUs
LSM9DS1 imu1;
LSM9DS1 imu2;

// unique BLE UUIDs generated from:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "2c616054-b134-4af3-8828-10cf8b206752"
#define CHARACTERISTIC_UUID "920eb6a5-4dd5-40e6-bd32-718fa583d8d6"
#define CHARACTERISTIC_UUID2 "9d025389-2a3f-47f9-aab8-333ad88e67da"
#define CHARACTERISTIC_UUID3 "a0208c9f-9763-409e-aa6c-6975fcc68cff"

//I2C addresses set on physical devices
#define LSM9DS1_M 0x1E 
#define LSM9DS1_AG  0x6B 
#define LSM9DS2_M  0x1C 
#define LSM9DS2_AG  0x6A

//ESP32 pin allocations for SDA/SCL
#define SDA_PIN 21    
#define SCL_PIN 22

// Define Pins for LED
#define R_LED 25
#define G_LED 27
#define B_LED 26

//#define PRINT_CALCULATED
#define PRINT_SPEED 1000 // time between polling
static unsigned long lastPrint = 0; // Keep track of polling time

// Earth's magnetic field varies by location. Toronto magnetic field found here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -10.17 // Declination (degrees) in Toronto,ON is -10.17 

//BLE Characteristics defined for IMU1, IMU2 and Differential Angle
BLECharacteristic customCharacteristic1(
  BLEUUID(CHARACTERISTIC_UUID), 
  BLECharacteristic::PROPERTY_READ | 
  BLECharacteristic::PROPERTY_WRITE
);
BLECharacteristic customCharacteristic2(
  BLEUUID(CHARACTERISTIC_UUID2), 
  BLECharacteristic::PROPERTY_READ | 
  BLECharacteristic::PROPERTY_WRITE
);
BLECharacteristic differentialAngle(
  BLEUUID(CHARACTERISTIC_UUID3), 
  BLECharacteristic::PROPERTY_READ | 
  BLECharacteristic::PROPERTY_WRITE
);

// This function handles the BLE server callbacks
bool deviceConnected = false;
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* MyServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* MyServer) {
      deviceConnected = false;
    }
};

struct IMU_OBJECT
{
    double yaw,pitch,roll;//yaw, pitch, and rollin Degrees
    double xV,yV,zV;//xyz vectors
    unsigned char yprD[6];//YPR assembled into char array, required for Bluetooth
};

//Function definitions
void startBle();
void startIMUs(); 
void updateIMUs();
IMU_OBJECT YPR_DegVec(float ax, float ay, float az, float mx, float my, float mz);
IMU_OBJECT convXYZtoCharArray(IMU_OBJECT IMU_YPR);
unsigned char diffAngle(IMU_OBJECT IMU1,IMU_OBJECT IMU2);

void setup() 
{
  Serial.begin(115200);
  
  pinMode(R_LED,OUTPUT);
  pinMode(G_LED,OUTPUT);
  pinMode(B_LED,OUTPUT);
  digitalWrite(R_LED, LOW);
  digitalWrite(G_LED, LOW);
  digitalWrite(B_LED, LOW);

  startBle(); //Initializing BLE Device and Characteristics
  startIMUs(); //Initialize both IMUs
  
}

void loop()
{
  unsigned char diffA[1];
  IMU_OBJECT IMU1_VecDeg,IMU2_VecDeg;
  
  // Update the sensor values whenever new data is available
  updateIMUs();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    
    Serial.println();
    Serial.println("IMU1");
    //Define characteristics of IMU1, yaw, pitch, roll, and xyz vectors for diff angle
    IMU1_VecDeg=YPR_DegVec(imu1.ax, imu1.ay, imu1.az, -imu1.my, -imu1.mx, imu1.mz);
    //Convert yaw, pitch, roll to char array
    IMU1_VecDeg=convXYZtoCharArray(IMU1_VecDeg);
    //load yaw, pitch, roll into bluetooth characteristic
    customCharacteristic1.setValue(IMU1_VecDeg.yprD,6); 

    Serial.println();
    Serial.println("IMU2");
    //Same process for IMU2
    IMU2_VecDeg=YPR_DegVec(imu2.ax, imu2.ay, imu2.az, -imu2.my, -imu2.mx, imu2.mz);
    IMU2_VecDeg=convXYZtoCharArray(IMU2_VecDeg);
    customCharacteristic2.setValue(IMU2_VecDeg.yprD,6);

    //Find Differential Angle between the 2 IMUs
    diffA[0]=diffAngle(IMU1_VecDeg,IMU2_VecDeg);
    differentialAngle.setValue(diffA,1);

    //Signals Bluetooth sent
    digitalWrite(G_LED, LOW);
    digitalWrite(B_LED, HIGH);
    delay(50);
    digitalWrite(B_LED, LOW);
    digitalWrite(G_LED, HIGH);

    lastPrint = millis(); // Update lastPrint time
  }
}
 void startBle()
 {
      Serial.println("Starting BLE work!");
      digitalWrite(B_LED, HIGH);
      delay(250);
      digitalWrite(B_LED, LOW);
      delay(250);
  
      BLEDevice::init("Periph_Device");
      BLEServer *pServer = BLEDevice::createServer();
      pServer->setCallbacks(new ServerCallbacks());  // Set the function that handles Server Callbacks
      BLEService *pService = pServer->createService(SERVICE_UUID);
      pService->addCharacteristic(&customCharacteristic1);
      pService->addCharacteristic(&customCharacteristic2);
      pService->addCharacteristic(&differentialAngle);

      digitalWrite(B_LED, HIGH);
      delay(250);
      digitalWrite(B_LED, LOW);
      delay(250);
  
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    
 }
 void startIMUs()
 {
  Wire.begin(SDA_PIN,SCL_PIN); 
  imu1.settings.device.commInterface = IMU_MODE_I2C;
  imu2.settings.device.commInterface = IMU_MODE_I2C;
  digitalWrite(R_LED, HIGH);
  delay(250);
  digitalWrite(R_LED, LOW);
  delay(250);
  if (!imu1.begin(LSM9DS1_AG,LSM9DS1_M))
  {
    Serial.println("Failed to communicate with IMU1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    digitalWrite(R_LED, HIGH);
    delay(250);
    while (1)
      ;
  }
  digitalWrite(R_LED, LOW);
  delay(250);
  digitalWrite(R_LED, HIGH);
  delay(250);
  digitalWrite(R_LED, LOW);
  delay(250);
  if (!imu2.begin(LSM9DS2_AG,LSM9DS2_M))
  {
    digitalWrite(R_LED, HIGH);
    Serial.println("Failed to communicate with IMU2.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
  digitalWrite(R_LED, LOW);
  digitalWrite(G_LED, HIGH);

 }
 void updateIMUs()
 {
  // Update the sensor values whenever new data is available

  if ( imu1.accelAvailable() )
  {
    imu1.readAccel();
  }
  if ( imu1.magAvailable() )
  {
    imu1.readMag();
  }
  if ( imu2.accelAvailable() )
  {
    imu2.readAccel();
  }
  if ( imu2.magAvailable() )
  {
    imu2.readMag();
  }
}

//Fill IMU Object with Pitch, Yaw, Roll, and xyz Vectors
IMU_OBJECT YPR_DegVec(float ax, float ay, float az, float mx, float my, float mz)
{
  IMU_OBJECT DegVecHeading;
  
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  float yaw;
  
  if (my == 0)
    yaw = (mx < 0) ? PI : 0;
  else
    yaw = atan2(mx, my);
    
  yaw -= DECLINATION * PI / 180;
  
  if (yaw > PI) yaw -= (2 * PI);
  else if (yaw < -PI) yaw += (2 * PI);  
  
  DegVecHeading.xV = cos(yaw)*cos(roll);
  DegVecHeading.yV = sin(yaw)*cos(roll);
  DegVecHeading.zV = sin(roll);

  // Convert everything from radians to degrees:
  yaw *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  DegVecHeading.yaw = yaw;
  DegVecHeading.pitch = pitch;
  DegVecHeading.roll = roll;
  Serial.print(pitch, 3);
  Serial.print(", ");
  Serial.print(roll, 3);
  Serial.print(", ");
  Serial.print(yaw, 3);
  Serial.print(", ");
  Serial.println();
 
  return  DegVecHeading;
}
//Convert yaw, pitch, roll to char array
IMU_OBJECT convXYZtoCharArray(IMU_OBJECT IMU_YPR)
{ 

  if(IMU_YPR.yaw>=0)
    {
      IMU_YPR.yprD[0]=0;//Positive Value
      IMU_YPR.yprD[1]=IMU_YPR.yaw;
    }
    else
    {
      IMU_YPR.yprD[0]=1;//negative Value
      IMU_YPR.yprD[1]=abs(IMU_YPR.yaw);
    }
    if(IMU_YPR.pitch>=0)
    {
      IMU_YPR.yprD[2]=0;//Positive Value
      IMU_YPR.yprD[3]=IMU_YPR.pitch;
    }
    else
    {
      IMU_YPR.yprD[2]=1;//negative Value
      IMU_YPR.yprD[3]=abs(IMU_YPR.pitch);
    }
    if(IMU_YPR.roll>=0)
    {
      IMU_YPR.yprD[4]=0;//Positive Value
      IMU_YPR.yprD[5]=IMU_YPR.roll;
    }
    else
    {
      IMU_YPR.yprD[4]=1;//negative Value
      IMU_YPR.yprD[5]=abs(IMU_YPR.roll);
    }
  return IMU_YPR;
}
//load yaw, pitch, roll into bluetooth characteristic
unsigned char diffAngle(IMU_OBJECT IMU1,IMU_OBJECT IMU2)
{
    unsigned char diffA;
    double diffAng;
    diffAng = (IMU1.xV*IMU2.xV) + (IMU1.yV*IMU2.yV) + (IMU1.zV*IMU2.zV);
    diffAng /= sqrt(pow(IMU1.xV,2)+pow(IMU1.yV,2)+pow(IMU1.zV,2));
    diffAng /= sqrt(pow(IMU2.xV,2)+pow(IMU2.yV,2)+pow(IMU2.zV,2));
    diffAng = acos(diffAng);
    diffAng = 180-(diffAng*180/M_PI);
    Serial.println(diffAng);
    return diffA = char(diffAng);
    
}
