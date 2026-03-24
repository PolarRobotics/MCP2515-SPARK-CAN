#pragma once
// #define LOOPBACK
// #define debug

#include <SPI.h>
#include <mcp2515.h>
#include <HardwareSerial.h>

#define SPI_CS_PIN 5
#define HALT_BUTTON 4
#define MOTOR_COUNT 8

extern MCP2515 mcp2515;
extern MCP2515::ERROR err;
extern uint8_t motors[MOTOR_COUNT];

// SPARK Packets
extern struct can_frame heartbeat;
#define heartbeat_id 0x2052C80
#define heartbeat_dlc 8
extern uint8_t heartbeat_data[8];

extern struct can_frame voltage_set; // -14V to 14V
#define voltage_set_id 0x2051080
#define voltage_set_dlc 4

extern struct can_frame velocity_set; // -1000 to 1000
#define velocity_set_id 0x2050480
#define velocity_set_dlc 8

extern struct can_frame position_set; // 0 to 360
#define position_set_id 0x2050C80
#define position_set_dlc 8

extern struct can_frame halt;
#define halt_id 0x0000040
#define halt_dlc 0

// Data for CAN Packets
uint8_t data[8*MOTOR_COUNT];
enum endianness{little, big};

// For GPIO Halt Pushbutton
#define HALT_BUTTON 4
extern unsigned long last_interrupt_time;
extern const unsigned long DEBOUNCE_DELAY;
extern bool halt_motors;

/// @brief Prints the error code returned by the MCP2515 library and the CAN frame data that was sent or received.
/// @param err The error code returned by the MCP2515 library
/// @param msg The CAN frame message that was sent or received
void printPacketWithError(MCP2515::ERROR err, can_frame *msg);

/// @brief Initalizes the CAN driver by resetting it, setting the bitrate, and setting the mode (loopback for testing, normal for actual use).
void initalizeCANDriver();

/// @brief  Sets up a CAN packet with the given parameters. This is a helper function to avoid repeating code when setting up different packets to send to the motor controllers.
/// @param packet A pointer to the can_frame struct that will be filled with the provided parameters. SPARK frames defined in CAN.h and CAN.cpp.
/// @param packet_id The ID of the packet (defined in CAN.h)
/// @param target_id The ID of the target motor controller
/// @param dlc The data length code (DLC) of the packet
/// @param data A pointer to the data to be sent in the packet
void setupPacket(can_frame *frame, uint32_t packet_id, uint32_t target_id, uint8_t dlc, uint8_t *data);

/// @brief Checks the state of the halt button (for testing with GPIO pushbutton).
void checkHalt();

/// @brief Pack 32-bit IEEE 754 float into an array of 8 bytes, with specified endianness and zero-padding
/// @param value The float value to pack
/// @param data A pointer to the array to fill with the packed data
/// @param endianness The endianness to use (little or big)
void packFloatIntoArray(double value, uint8_t* data, endianness);

/// @brief Attempts to send a CAN packet from the MCP2515 using the provided parameters.
/// @param data 32-bit IEEE 754 float value packed into an array of 8 bytes
/// @param target_id The ID of the target motor controller
/// @param frame The SPARK CAN frame type to send
/// @param packet_id The ID of the packet
/// @param dlc The data length code (DLC) of the packet
void sendPacket(uint8_t* data, uint32_t target_id, can_frame *frame, uint32_t packet_id, uint8_t dlc);

/// @brief Attempts to read a CAN packet from the MCP2515. If a packet is received, it is stored in the provided can_frame struct.
/// @param frame 
void readPacket(can_frame *frame);

