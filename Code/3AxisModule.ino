#include <Wire.h> //Contect to Sensor


// IC2 Address of the magnometer
#define I2C_Freq 100000
#define HMC5883L_Address 0x1E

#define SDA_0 21
#define SCL_0 22
TwoWire I2C_0 = TwoWire(0);

void setup() {

    Serial.begin(9600);
    Serial.println("Testing");
      I2C_0.begin(SDA_0 , SCL_0 , I2C_Freq); 
      
    Wire.begin();
  /* initiaise the module */
  Int_HMC5883L();



}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(HMC5883L)
  


  

}
