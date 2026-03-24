/**
 * @brief Implements functions for Non-FRC SPARK Motor Controller Communication using an MCP2515 CAN controller.
 * @author Quentin Osterhage
 * Adapted from: https://github.com/autowp/arduino-mcp2515
 */

#include "CAN.h"
#define debug

/**
 * @brief 
 * Motor Controller Layout                     
 * TOP: Driving controller ID
 * BOTTOM: Turning controller ID
 *
 *                  ^                                            
 *                  | Fwd                                  
 *       ___________________________            
 *      |   ____             ____   |          
 *      |  |LF  |           |RF  |  |            
 *      |  |0x00|           |0x00|  |   
 *      |  |0x00|           |0x00|  |
 *      |  |____|           |____|  |          
 *      |           .               |          
 *      |   ____             ____   |         
 *      |  |LB  |           |RB  |  |          
 *      |  |0x00|           |0x00|  |  
 *      |  |0x00|           |0x00|  |              
 *      |  |____|           |____|  |          
 *      |___________________________|          
 * 
 */

// Define motor IDs here
uint8_t motors[MOTOR_COUNT] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

MCP2515 mcp2515(SPI_CS_PIN);
MCP2515::ERROR err;

struct can_frame heartbeat;
uint8_t heartbeat_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct can_frame halt;
struct can_frame velocity_set;
struct can_frame position_set;
struct can_frame voltage_set;     

// Halt Button debounce (for testing, remove latter)
unsigned long last_interrupt_time = 0;
const unsigned long DEBOUNCE_DELAY = 200;
bool halt_motors = false;

void initalizeCANDriver(){
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  #ifdef LOOPBACK
  mcp2515.setLoopbackMode();
  #else
  mcp2515.setNormalMode();
  #endif
}

void setupPacket(can_frame *frame, uint32_t packet_id, uint32_t target_id, uint8_t dlc, uint8_t *data){
  frame->can_id = packet_id + target_id | CAN_EFF_FLAG;
  frame->can_dlc = dlc;
  for(int i = 0; i < dlc; i++){
    frame->data[i] = data[i];
  }
}

void printPacketWithError(MCP2515::ERROR err, can_frame *msg){
  switch(err){
    case MCP2515::ERROR_OK:
      Serial.printf("[Packet Sent]:\nID 0x%X\nData: " , msg->can_id & CAN_EFF_MASK);
      for(int i = 0; i < msg->can_dlc; i++){
        Serial.printf("0x%x ", msg->data[i]);
      }
      Serial.println();
      break;
    case MCP2515::ERROR_FAIL:
      Serial.printf("Failed");
      break;
    case MCP2515::ERROR_ALLTXBUSY:
      Serial.printf("All TX buffers are busy\n");
      initalizeCANDriver();
      break;
    case MCP2515::ERROR_FAILINIT:
      Serial.printf("Failed to initialize\n");
      break;
    case MCP2515::ERROR_FAILTX:
      Serial.printf("Failed to transmit\n");
      break;
    case MCP2515::ERROR_NOMSG:
      Serial.printf("No message received\n");
      break;
    default:
      Serial.println("Unknown error");
  }
}

// For testing w/ GPIO pushbutton
void checkHalt(){
  if (digitalRead(HALT_BUTTON) == LOW) {
    // Check if enough time has passed since the last toggle
    if (millis() - last_interrupt_time > DEBOUNCE_DELAY) {
      halt_motors = !halt_motors; // Toggle the state
      if (halt_motors) {
        err = mcp2515.sendMessage(&halt);
        #ifdef debug
        printPacketWithError(err, &halt);
        #endif
      } else {
        Serial.println("Resume control");
      }
      last_interrupt_time = millis();
    }
  }
}


void packFloatIntoArray(double value, uint8_t* data, endianness){
  uint32_t bits;
  float float_value = static_cast<float>(value); // Convert double to float for 4 bytes
  std::memcpy(&bits, &float_value, sizeof(bits));

  switch(endianness)
  {
    case big:
      // Big-endian (MSB first)
      data[0] = (bits >> 24) & 0xFF;
      data[1] = (bits >> 16) & 0xFF;
      data[2] = (bits >> 8)  & 0xFF;
      data[3] =  bits        & 0xFF;
      break;
    case little:
      // Little-endian (LSB first)
      data[0] =  bits        & 0xFF;
      data[1] = (bits >> 8)  & 0xFF;
      data[2] = (bits >> 16) & 0xFF;
      data[3] = (bits >> 24) & 0xFF;
      break;
  }
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;
}

void sendPacket(uint8_t* data, uint32_t target_id, can_frame *frame, uint32_t packet_id, uint8_t dlc){
  setupPacket(frame, packet_id, target_id, dlc, data);
  mcp2515.sendMessage(frame);
}

void readPacket(can_frame *frame){
  err = mcp2515.readMessage(frame);
  printPacketWithError(err, frame);
}