/*****************************************************************

*****************************************************************/

#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu1;
LSM9DS1 imu2;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "2c616054-b134-4af3-8828-10cf8b206752"
#define CHARACTERISTIC_UUID "920eb6a5-4dd5-40e6-bd32-718fa583d8d6"
#define CHARACTERISTIC_UUID2 "9d025389-2a3f-47f9-aab8-333ad88e67da"
#define CHARACTERISTIC_UUID3 "a0208c9f-9763-409e-aa6c-6975fcc68cff"

#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define LSM9DS2_M  0x1C // Would be 0x1C if SDO_M is HIGH
#define LSM9DS2_AG  0x6A // Would be 0x6A if SDO_AG is HIGH

#define SDA_PIN 21    //ESP32 pin allocations for SDA/SCL
#define SCL_PIN 22

// Define Pins for LED
#define R_LED 25
#define G_LED 27
#define B_LED 26

#define PRINT_CALCULATED
#define PRINT_SPEED 1000 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. 
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -10.17 // Declination (degrees) in Toronto,ON is -10.17 

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
/* This function handles the server callbacks */
bool deviceConnected = false;
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* MyServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* MyServer) {
      deviceConnected = false;
    }
};
struct HEADING
{
    double x,y,z;
    unsigned char xyz[6];
};

//Function definitions
void updateIMUs ();
unsigned char convXYZtoCharArray(HEADING IMU_YPR);
HEADING YPR(float ax, float ay, float az, float mx, float my, float mz);
HEADING YPR_vector(float ax, float ay, float az, float mx, float my, float mz);



void setup() 
{
   pinMode(R_LED,OUTPUT);
   pinMode(G_LED,OUTPUT);
   pinMode(B_LED,OUTPUT);
   digitalWrite(R_LED, LOW);
   digitalWrite(G_LED, LOW);
   digitalWrite(B_LED, LOW);

   
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
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.begin(115200);
  Wire.begin(SDA_PIN,SCL_PIN);  //Change pin allocation in I2C library
  
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

void loop()
{
  
  HEADING IMU1_YPR,IMU2_YPR,IMU1_VEC,IMU2_VEC;
  double diffAngle;
  
  // Update the sensor values whenever new data is available
  void updateIMUs ();

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    unsigned char xyz1[6];
    unsigned char xyz2[6];
    unsigned char diffA[1];
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
    Serial.println();
    Serial.println("IMU1");
    IMU1_YPR=YPR(imu1.ax, imu1.ay, imu1.az, -imu1.my, -imu1.mx, imu1.mz);
    IMU1_VEC=YPR_vector(imu1.ax, imu1.ay, imu1.az, -imu1.my, -imu1.mx, imu1.mz);
    if(IMU1_YPR.x>=0)
    {
      xyz1[0]=0;//Positive Value
      xyz1[1]=IMU1_YPR.x;
    }
    else
    {
      xyz1[0]=1;//negative Value
      xyz1[1]=abs(IMU1_YPR.x);
    }
    if(IMU1_YPR.y>=0)
    {
      xyz1[2]=0;//Positive Value
      xyz1[3]=IMU1_YPR.y;
    }
    else
    {
      xyz1[2]=1;//negative Value
      xyz1[3]=abs(IMU1_YPR.y);
    }
    if(IMU1_YPR.z>=0)
    {
      xyz1[4]=0;//Positive Value
      xyz1[5]=IMU1_YPR.z;
    }
    else
    {
      xyz1[4]=1;//negative Value
      xyz1[5]=abs(IMU1_YPR.y);
    }
    customCharacteristic1.setValue(xyz1,6);

    Serial.println();
    Serial.println("IMU2");
    IMU2_YPR=YPR(imu2.ax, imu2.ay, imu2.az, -imu2.my, -imu2.mx, imu2.mz);
    IMU2_VEC=YPR_vector(imu2.ax, imu2.ay, imu2.az, -imu2.my, -imu2.mx, imu2.mz);
    
    
    //xyz2=convXYZtoCharArray(IMU2_YPR);
    xyz2=convXYZtoCharArray(IMU2_YPR);
    customCharacteristic2.setValue(xyz2,6);

    
    diffAngle = (IMU1_VEC.x*IMU2_VEC.x) + (IMU1_VEC.y*IMU2_VEC.y) + (IMU1_VEC.z*IMU2_VEC.z);
    diffAngle /= sqrt(pow(IMU1_VEC.x,2)+pow(IMU1_VEC.y,2)+pow(IMU1_VEC.z,2));
    diffAngle /= sqrt(pow(IMU2_VEC.x,2)+pow(IMU2_VEC.y,2)+pow(IMU2_VEC.z,2));
    diffAngle = acos(diffAngle);
    diffA[0]= char(diffAngle*180/M_PI);
    Serial.println(diffAngle*180/M_PI);
    differentialAngle.setValue(diffA,1);
    
    digitalWrite(G_LED, LOW);
    digitalWrite(B_LED, HIGH);
    delay(50);
    digitalWrite(B_LED, LOW);
    digitalWrite(G_LED, HIGH);

    
    lastPrint = millis(); // Update lastPrint time
  }
}
 void updateIMUs ()
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
unsigned char convXYZtoCharArray(HEADING IMU_YPR)
{ 
  unsigned char xyz[6];
  if(IMU_YPR.x>=0)
    {
      IMU_YPR.xyz[0]=0;//Positive Value
      IMU_YPR.xyz[1]=IMU_YPR.x;
    }
    else
    {
      IMU_YPR.xyz[0]=1;//negative Value
      IMU_YPR.xyz[1]=abs(IMU_YPR.x);
    }
    if(IMU_YPR.y>=0)
    {
      IMU_YPR.xyz[2]=0;//Positive Value
      IMU_YPR.xyz[3]=IMU_YPR.y;
    }
    else
    {
      IMU_YPR.xyz[2]=1;//negative Value
      IMU_YPR.xyz[3]=abs(IMU_YPR.y);
    }
    if(IMU_YPR.z>=0)
    {
      IMU_YPR.xyz[4]=0;//Positive Value
      IMU_YPR.xyz[5]=IMU_YPR.z;
    }
    else
    {
      IMU_YPR.xyz[4]=1;//negative Value
      IMU_YPR.xyz[5]=abs(IMU_YPR.y);
    }
    return xyz;
}
HEADING YPR(float ax, float ay, float az, float mx, float my, float mz)
{
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
  
  // Convert everything from radians to degrees:
  yaw *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  
  Serial.print(pitch, 3);
  Serial.print(", ");
  Serial.print(roll, 3);
  Serial.print(", ");
  Serial.print(yaw, 3);
  Serial.print(", ");

  HEADING YawPitchRoll = {yaw, pitch, roll};
  
  return YawPitchRoll;
}
HEADING YPR_vector(float ax, float ay, float az, float mx, float my, float mz)
{
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
  
//  Serial.print(pitch, 3);
//  Serial.print(", ");
//  Serial.print(roll, 3);
//  Serial.print(", ");
//  Serial.print(yaw, 3);
//  Serial.print(", ");


  HEADING VectorHeading = { cos(yaw)*cos(roll), sin(yaw)*cos(roll), sin(roll) };

  
  return VectorHeading;
}
