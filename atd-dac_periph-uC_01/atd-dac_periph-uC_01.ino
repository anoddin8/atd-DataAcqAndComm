/*****************************************************************

*****************************************************************/

#include <Wire.h>
#include <SparkFunLSM9DS1.h>

// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu1;
LSM9DS1 imu2;

#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define LSM9DS2_M  0x1C // Would be 0x1C if SDO_M is HIGH
#define LSM9DS2_AG  0x6A // Would be 0x6A if SDO_AG is HIGH

#define SDA_PIN 21    //ESP32 pin allocations for SDA/SCL
#define SCL_PIN 22

#define PRINT_CALCULATED
#define PRINT_SPEED 10 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//Function definitions
void printGyro();  
void printAccel(); 
void printMag();   
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);


void setup() 
{
  
  Serial.begin(115200);
  Wire.begin(SDA_PIN,SCL_PIN);  //Change pin allocation in I2C library
  
  imu1.settings.device.commInterface = IMU_MODE_I2C;
  imu2.settings.device.commInterface = IMU_MODE_I2C;
  
  if (!imu1.begin(LSM9DS1_AG,LSM9DS1_M))
  {
    Serial.println("Failed to communicate with IMU1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }

  if (!imu2.begin(LSM9DS2_AG,LSM9DS2_M))
  {
    Serial.println("Failed to communicate with IMU2.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

void loop()
{
  // Update the sensor values whenever new data is available
  if ( imu1.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu1.readGyro();
  }
  if ( imu1.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu1.readAccel();
  }
  if ( imu1.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu1.readMag();
  }
  //************
   // Update the sensor values whenever new data is available
  if ( imu2.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu2.readGyro();
  }
  if ( imu2.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu2.readAccel();
  }
  if ( imu2.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu2.readMag();
  }
  if ((lastPrint + PRINT_SPEED) < millis())
  {
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
    Serial.println();
    Serial.println("IMU1");
    printAttitude(imu1.ax, imu1.ay, imu1.az, -imu1.my, -imu1.mx, imu1.mz);
    Serial.println();
    Serial.println("IMU2");
    printAttitude(imu2.ax, imu2.ay, imu2.az, -imu2.my, -imu2.mx, imu2.mz);
    
    delay(5000);
    
    lastPrint = millis(); // Update lastPrint time
  }
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  float heading;
  
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  
  Serial.print(pitch, 3);
  Serial.print(", ");
  Serial.print(roll, 3);
  Serial.print(", ");
  Serial.print(heading, 3);
  Serial.print(", ");
}
