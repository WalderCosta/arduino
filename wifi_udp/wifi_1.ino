int ii;
void setup()
{
  Serial.begin(9600);
  ii=0;
}
void loop()
{
  delay(1000);
  Serial.print("packet # ");
  Serial.print(ii++,DEC);
  Serial.println(" - hello ESP8266 WiFi"); //output the serial data
}
