#include <Arduino.h>
#include <HardwareSerial.h>
#include <ps5Controller.h>

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "Pairing/pairing.h"
#include "Motor_Communication/CAN.h"

#include <cstring>

#define SPI_CS_PIN 5
#define HALT_BUTTON 4

/* Bluetooth Setup stuff*/
// Prototypes for Controller Callbacks
// Implementations located at the bottom of this file
void onConnection();
void onDisconnect();
bool initBluetooth();
void printDeviceAddress();

// Obtain PS5 controller inputs here
void getInputs();

void setup(){
  Serial.begin(115200);
  pinMode(HALT_BUTTON, INPUT_PULLUP);

  initalizeCANDriver();
  setupPacket(&heartbeat, heartbeat_id, 0, heartbeat_dlc, heartbeat_data);
  setupPacket(&halt, halt_id, 0, halt_dlc, NULL);

  initBluetooth();
  printDeviceAddress();
  activatePairing();
  ps5.attachOnConnect(onConnection);
  ps5.attachOnDisconnect(onDisconnect);
}

void loop() {
  if(ps5.isConnected()){
    getInputs();

    if(!halt_motors){
      //sendPacket()
      //readPacket()
    }
    checkHalt();
  }
  else{
    mcp2515.sendMessage(&halt);
  }
}

void getInputs(){
  // Get PS5 controller inputs here
}

bool initBluetooth()
{
  if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}

void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
 
  for (int i = 0; i < 6; i++) {
 
    char str[3];
 
    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);
 
    if (i < 5){
      Serial.print(":");
    }
 
  }
}

// Bluetoooth controller connection check
void onConnection() {
  if (ps5.isConnected()) {
    Serial.println(F("Controller Connected."));
    // ps5.setLed(0, 255, 0);   // set LED green
    //lights.setLEDStatus(Lights::PAIRED);
  }
}

/**
 * @brief onDisconnect: Function to be called on controller disconnect
 * Stops bots from driving off and ramming into a wall or someone's foot if they disconnect
 */
void onDisconnect() {
    Serial.println(F("Controller Disconnected."));
}
