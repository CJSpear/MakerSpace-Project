//Phantasm Full Setup

#include <Wire.h> //I2C Arduino Library

#include "TinyGPS++.h"
#include "SoftwareSerial.h" //GPS Serials

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

#include "HyperDisplay_UG2856KLBAG01.h"  //OLED Display
#define SERIAL_PORT Serial  
#define SPI_PORT SPI   
#define CS_PIN 4
#define DC_PIN 5     
#define USE_SPI 1           // Choose your interface. 0 = I2C, 1 = SPI
UG2856KLBAG01_SPI myTOLED;  // Declare a SPI-based Transparent OLED object called myTOLED

//Altimeter
#include <Adafruit_MPL3115A2.h>
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

//Barrometer
#include <Adafruit_BMP085.h>

//BMI160 Gryoscope
#include <BMI160Gen.h>

SoftwareSerial serial_connection(12, 14); //RX=pin 12, TX=pin 14
int pinSDA = 26;      
int pinSCL = 27;

// BME280 I2C address is 0x76(108)
#define Addr 0x76


void setup() {
  Serial.begin(9600);
  Wire.begin(26,27);

  //HMC5883L - 3Axis Digital Compass 
  Wire.beginTransmission(address);
  Wire.write(0x02); //Select Mode Register
  Wire.write(0x00); // Continuous Measurement Mode
  Wire.endTransmission();

  //OLED Setup
  SPI_PORT.begin(18, -1, 32, 4);  
  myTOLED.begin(CS_PIN, DC_PIN, SPI_PORT);

  //BMI160 Gryo
  BMI160.begin(BMI160GenClass::SPI_MODE, /*SPI PIN*/ 11);
  uint8_t dev_id = BMI160.getDeviceID();
  //Set Accelerometer range to 250degrees/Second
  BMI160.setGyroRange(250);


  //GPS Starting Routine
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();

  while (*gpsStream)
    if (gps.encode(*gpsStream++))
      displayInfo();

  Serial.println();
  Serial.println(F("GPS Starting."));
  
}

//For Later, Looping System for Auto Updates
void loop() {  
  compass();
  delay(250);
}


//Find XYZ Axis Data
void compass (){
    int x,y,z; //triple axis data

  
  Wire.beginTransmission(address);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
}

//Find GPS Location & Display
void gps(){
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();


   
}
  

void barom (void){
  Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");
    
    Serial.println();
    delay(500);
  
}

//XYZ Gyro
void BMI (){
  int gxRaw, gyRaw, gzRaw;
  float gx, gy, gz;

  // read raw gyro measurements from device
  BMI160.readGyro(gxRaw, gyRaw, gzRaw);

  // convert the raw gyro data to degrees/second
  gx = convertRawGyro(gxRaw);
  gy = convertRawGyro(gxRaw);
  gz = convertRawGyro(gxRaw);
  
}


//Convert Gyro
float convertRawGyro(int gRaw){
   // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}


//Barometer
float pascals = baro.getPressure(); {
  // Use http://www.onlineconversion.com/pressure.htm for other units
  //Pascals in Milimeter (Mercury (HG)) (pascals/133);
  //Return Altitude in Meters
  float altm = baro.getAltitude();
  //Return Temperature in C*
  float tempC = baro.getTemperature();
}


//
void BME (){
  unsigned int b1[24];
unsigned int data[8];
unsigned int dig_H1 = 0;
for(int i = 0; i < 24; i++)
{
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select data register
Wire.write((136+i));
// Stop I2C Transmission
Wire.endTransmission();
 
// Request 1 byte of data
Wire.requestFrom(Addr, 1);
 
// Read 24 bytes of data
if(Wire.available() == 1)
{
b1[i] = Wire.read();
}
}
 
// Convert the data
// temp coefficients
unsigned int dig_T1 = (b1[0] & 0xff) + ((b1[1] & 0xff) * 256);
int dig_T2 = b1[2] + (b1[3] * 256);
int dig_T3 = b1[4] + (b1[5] * 256);
 
// pressure coefficients
unsigned int dig_P1 = (b1[6] & 0xff) + ((b1[7] & 0xff ) * 256);
int dig_P2 = b1[8] + (b1[9] * 256);
int dig_P3 = b1[10] + (b1[11] * 256);
int dig_P4 = b1[12] + (b1[13] * 256);
int dig_P5 = b1[14] + (b1[15] * 256);
int dig_P6 = b1[16] + (b1[17] * 256);
int dig_P7 = b1[18] + (b1[19] * 256);
int dig_P8 = b1[20] + (b1[21] * 256);
int dig_P9 = b1[22] + (b1[23] * 256);
 
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select data register
Wire.write(161);
// Stop I2C Transmission
Wire.endTransmission();
 
// Request 1 byte of data
Wire.requestFrom(Addr, 1);
 
// Read 1 byte of data
if(Wire.available() == 1)
{
dig_H1 = Wire.read();
}
 
for(int i = 0; i < 7; i++)
{
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select data register
Wire.write((225+i));
// Stop I2C Transmission
Wire.endTransmission();
 
// Request 1 byte of data
Wire.requestFrom(Addr, 1);
 
// Read 7 bytes of data
if(Wire.available() == 1)
{
b1[i] = Wire.read();
}
}
 
// Convert the data
// humidity coefficients
int dig_H2 = b1[0] + (b1[1] * 256);
unsigned int dig_H3 = b1[2] & 0xFF ;
int dig_H4 = (b1[3] * 16) + (b1[4] & 0xF);
int dig_H5 = (b1[4] / 16) + (b1[5] * 16);
int dig_H6 = b1[6];
 
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select control humidity register
Wire.write(0xF2);
// Humidity over sampling rate = 1
Wire.write(0x01);
// Stop I2C Transmission
Wire.endTransmission();
 
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select control measurement register
Wire.write(0xF4);
// Normal mode, temp and pressure over sampling rate = 1
Wire.write(0x27);
// Stop I2C Transmission
Wire.endTransmission();
 
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select config register
Wire.write(0xF5);
// Stand_by time = 1000ms
Wire.write(0xA0);
// Stop I2C Transmission
Wire.endTransmission();
 
for(int i = 0; i < 8; i++)
{
// Start I2C Transmission
Wire.beginTransmission(Addr);
// Select data register
Wire.write((247+i));
// Stop I2C Transmission
Wire.endTransmission();
 
// Request 1 byte of data
Wire.requestFrom(Addr, 1);
 
// Read 8 bytes of data
if(Wire.available() == 1)
{
data[i] = Wire.read();
}
}
 
// Convert pressure and temperature data to 19-bits
long adc_p = (((long)(data[0] & 0xFF) * 65536) + ((long)(data[1] & 0xFF) * 256) + (long)(data[2] & 0xF0)) / 16;
long adc_t = (((long)(data[3] & 0xFF) * 65536) + ((long)(data[4] & 0xFF) * 256) + (long)(data[5] & 0xF0)) / 16;
// Convert the humidity data
long adc_h = ((long)(data[6] & 0xFF) * 256 + (long)(data[7] & 0xFF));
 
// Temperature offset calculations
double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
(((double)adc_t)/131072.0 - ((double)dig_T1)/8192.0)) * ((double)dig_T3);
double t_fine = (long)(var1 + var2);
double cTemp = (var1 + var2) / 5120.0;
double fTemp = cTemp * 1.8 + 32;
 
// Pressure offset calculations
var1 = ((double)t_fine / 2.0) - 64000.0;
var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
var2 = var2 + var1 * ((double)dig_P5) * 2.0;
var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
double p = 1048576.0 - (double)adc_p;
p = (p - (var2 / 4096.0)) * 6250.0 / var1;
var1 = ((double) dig_P9) * p * p / 2147483648.0;
var2 = p * ((double) dig_P8) / 32768.0;
double pressure = (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100;
 
// Humidity offset calculations
double var_H = (((double)t_fine) - 76800.0);
var_H = (adc_h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_H)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_H * (1.0 + dig_H3 / 67108864.0 * var_H)));
double humidity = var_H * (1.0 - dig_H1 * var_H / 524288.0);
if(humidity > 100.0)
{
humidity = 100.0;
}
else if(humidity < 0.0)
{
humidity = 0.0;
}
 
// Output data to serial monitor
Serial.print("Temperature in Celsius : ");
Serial.print(cTemp);
Serial.println(" C");
Serial.print("Temperature in Fahrenheit : ");
Serial.print(fTemp);
Serial.println(" F");
Serial.print("Pressure : ");
Serial.print(pressure);
Serial.println(" hPa");
Serial.print("Relative Humidity : ");
Serial.print(humidity);
Serial.println(" RH");
delay(1000);
}






}
}
