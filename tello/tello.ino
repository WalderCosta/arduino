#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

//const char* WIFI_ID = "werneckpaiva";
// const char WIFI_PASS = "";
//const char* TELLO_IP = "192.168.0.16";

const char* WIFI_ID = "TELLO-WP";
const char* WIFI_PASS = NULL;

const char* TELLO_IP = "192.168.10.1";
const int TELLO_PORT = 8889;
const int TELLO_STATE_PORT = 8890;
  
WiFiUDP udpCommand;
WiFiUDP udpStatus;

void setup() {
  Serial.begin(74880);

  WiFi.hostname("WeMos");
  WiFi.begin();

  // Wait until we're connected.
  Serial.print("Connecting to WiFi: ");
  Serial.print(WIFI_ID);
  WiFi.begin(WIFI_ID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" connected");
  Serial.println(WiFi.localIP());

  Serial.print("Listening status on port ");
  Serial.println(TELLO_STATE_PORT);
  udpStatus.begin(TELLO_STATE_PORT);
}

bool sendCommand(char *cmd){
  int packetSize = 0;
  do{
    Serial.print("Sending '");
    Serial.print(cmd);
    Serial.println("'");

    udpCommand.beginPacket(TELLO_IP, TELLO_PORT);
    udpCommand.write(cmd);
    udpCommand.endPacket();

    udpCommand.begin(udpCommand.localPort());
    
    for (byte i=0; i<20; i++){
      packetSize = udpCommand.parsePacket();
      if (packetSize > 0){
        break;
      }
      delay(500);
    }
  } while(packetSize == 0);

  char reply[packetSize + 1];
  udpCommand.read(reply, packetSize);
  reply[packetSize] = '\0';
  Serial.print(packetSize);
  Serial.print(" *");
  Serial.print(reply);
  Serial.println("*");
  return (strcmp(reply, "ok") == 0);
}


char* commands[2] = {"command", "battery?"};

bool isOk = false;

int currentCommand = 0;
int retries = 0;

void loop() {
  
  if (currentCommand < 2){
    isOk = sendCommand(commands[currentCommand]);
    retries++;
    if (isOk || retries > 3){
      currentCommand++;
      isOk = false;
      retries = 0;
    } else {
      Serial.println("Command failed");
    }
  } else {
    int statusPacketSize = udpStatus.parsePacket();
    if (statusPacketSize > 0){
        char reply[statusPacketSize + 1];
        udpStatus.read(reply, statusPacketSize);
        reply[statusPacketSize] = '\0';
        Serial.println(reply);
    }
    
    sendCommand("battery?");
  }
}
