
void setup() {
  Serial.begin(115200);

  // put your setup code here, to run once:

}


void loop() {
  Serial.println("-------------");
  Serial.print("MOSI");
  Serial.println(MOSI);
  Serial.print("MISO");
  Serial.println(MISO);
  Serial.print("SCK");
  Serial.println(SCK);
  Serial.print("SS");
  Serial.println(SS);
  Serial.print("SDA");
  Serial.println(SDA);
  Serial.print("SCL");
  Serial.println(SCL);
  delay(100000);
 

  

  // put your main code here, to run repeatedly:

}
