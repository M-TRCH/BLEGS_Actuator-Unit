/**
 * @file    protocol.h
 * @brief   High-Speed Binary Communication Protocol (Hybrid Mode)
 * @details Supports both legacy ASCII commands and new binary protocol
 *          based on HSPUP specification with CRC-16 integrity checking
 * @author  M-TRCH
 * @date    2025-12-03
 * @version 1.0
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// Protocol constants
#define PROTOCOL_HEADER_1           0xFE        // First sync byte
#define PROTOCOL_HEADER_2           0xEE        // Second sync byte
#define PROTOCOL_MAX_PAYLOAD_SIZE   32          // Maximum payload size in bytes
#define PROTOCOL_TIMEOUT_MS         50          // Packet reception timeout

// Packet Type IDs
enum PacketType : uint8_t {
    PKT_CMD_SET_GOAL        = 0x01,     // Command: Set position goal (PC -> Motor)
    PKT_CMD_SET_CONFIG      = 0x02,     // Command: Set configuration (PC -> Motor)
    PKT_CMD_PING            = 0x03,     // Command: Ping/health check
    
    PKT_FB_STATUS           = 0x81,     // Feedback: Status response (Motor -> PC)
    PKT_FB_CONFIG           = 0x82,     // Feedback: Configuration response
    PKT_FB_ERROR            = 0x83,     // Feedback: Error report
    PKT_FB_PONG             = 0x84      // Feedback: Pong response
};

// Control Mode IDs (used in CMD_SET_GOAL payload)
enum ControlMode : uint8_t {
    MODE_DIRECT_POSITION    = 0x00,     // Direct position control (PID only)
    MODE_SCURVE_PROFILE     = 0x01      // S-Curve profile motion
};

// Status Flags (bitfield for FB_STATUS)
enum StatusFlags : uint8_t {
    STATUS_MOVING           = (1 << 0), // Bit 0: Motor is moving
    STATUS_ERROR            = (1 << 1), // Bit 1: Error occurred
    STATUS_AT_GOAL          = (1 << 2), // Bit 2: At goal position
    STATUS_OVERHEAT         = (1 << 3), // Bit 3: Overheating
    STATUS_OVERCURRENT      = (1 << 4), // Bit 4: Overcurrent detected
    STATUS_ENCODER_ERROR    = (1 << 5)  // Bit 5: Encoder error
};

// Error Codes
enum ErrorCode : uint8_t {
    ERR_NONE                = 0x00,
    ERR_CRC_FAILED          = 0x01,
    ERR_INVALID_PACKET      = 0x02,
    ERR_TIMEOUT             = 0x03,
    ERR_UNKNOWN_COMMAND     = 0x04,
    ERR_INVALID_PAYLOAD     = 0x05,
    ERR_MOTOR_FAULT         = 0x06
};

// Generic binary packet structure
struct __attribute__((packed)) BinaryPacket {
    uint8_t header[2];              // [0xFE, 0xEE]
    uint8_t packet_type;            // PacketType enum
    uint8_t payload_len;            // Length of payload (0-32)
    uint8_t payload[PROTOCOL_MAX_PAYLOAD_SIZE];
    uint16_t crc16;                 // CRC-16-IBM (Little-endian)
};

// Payload structures for CMD_SET_GOAL
struct __attribute__((packed)) PayloadDirectPosition {
    int32_t target_pos;             // Target position (encoder ticks or degrees*100)
};

struct __attribute__((packed)) PayloadSCurveProfile {
    int32_t target_pos;             // Target position (encoder ticks or degrees*100)
    uint16_t duration_ms;           // Duration in milliseconds
};

// Union for CMD_SET_GOAL payload variants
struct __attribute__((packed)) PayloadSetGoal {
    uint8_t control_mode;           // ControlMode enum
    union {
        PayloadDirectPosition direct;
        PayloadSCurveProfile scurve;
    } data;
};

// Payload structure for FB_STATUS
struct __attribute__((packed)) PayloadStatus {
    int32_t actual_pos;             // Current position (encoder ticks or degrees*100)
    int16_t actual_current;         // Current in mA (0 if not available)
    uint8_t status_flags;           // StatusFlags bitfield
};

// Payload structure for FB_ERROR
struct __attribute__((packed)) PayloadError {
    uint8_t error_code;             // ErrorCode enum
    uint8_t last_packet_type;       // Packet type that caused error
    uint16_t debug_info;            // Additional debug information
};

// Payload structure for CMD_SET_CONFIG
struct __attribute__((packed)) PayloadSetConfig {
    uint8_t config_id;              // Configuration parameter ID
    float value;                    // Configuration value
};

// Function prototypes

/**
 * @brief Initialize the protocol system
 */
void protocolInit();

/**
 * @brief Calculate CRC-16-IBM for data buffer
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @return CRC-16 value
 */
uint16_t calculateCRC16(const uint8_t* data, uint16_t length);

/**
 * @brief Update CRC-16 with a single byte (for incremental calculation)
 * @param crc Current CRC value
 * @param data Byte to add to CRC
 * @return Updated CRC value
 */
uint16_t crc16Update(uint16_t crc, uint8_t data);

/**
 * @brief Verify CRC of a received packet
 * @param pkt Pointer to BinaryPacket structure
 * @return true if CRC is valid, false otherwise
 */
bool verifyCRC16(const BinaryPacket* pkt);

/**
 * @brief Calculate and set CRC for a packet before sending
 * @param pkt Pointer to BinaryPacket structure
 */
void setCRC16(BinaryPacket* pkt);

/**
 * @brief Check if binary packet is available on serial port
 * @param serial Pointer to HardwareSerial object
 * @return true if binary packet header detected, false otherwise
 */
bool isBinaryPacketAvailable(HardwareSerial* serial);

/**
 * @brief Receive and parse a binary packet from serial port
 * @param serial Pointer to HardwareSerial object
 * @param pkt Pointer to BinaryPacket structure to fill
 * @param timeout_ms Timeout in milliseconds
 * @return true if packet received and CRC valid, false otherwise
 */
bool receivePacket(HardwareSerial* serial, BinaryPacket* pkt, uint32_t timeout_ms = PROTOCOL_TIMEOUT_MS);

/**
 * @brief Send a binary packet via serial port
 * @param serial Pointer to HardwareSerial object
 * @param pkt Pointer to BinaryPacket structure to send
 */
void sendPacket(HardwareSerial* serial, const BinaryPacket* pkt);

/**
 * @brief Send a status feedback packet
 * @param serial Pointer to HardwareSerial object
 * @param pos Current position
 * @param current Current in mA
 * @param flags Status flags
 */
void sendStatusFeedback(HardwareSerial* serial, int32_t pos, int16_t current, uint8_t flags);

/**
 * @brief Send an error feedback packet
 * @param serial Pointer to HardwareSerial object
 * @param error_code Error code
 * @param last_pkt_type Packet type that caused error
 * @param debug_info Additional debug info
 */
void sendErrorFeedback(HardwareSerial* serial, uint8_t error_code, uint8_t last_pkt_type = 0, uint16_t debug_info = 0);

/**
 * @brief Process received CMD_SET_GOAL packet
 * @param payload Pointer to PayloadSetGoal structure
 * @param target_pos Output: target position
 * @param duration_ms Output: duration (only for S-Curve mode)
 * @param mode Output: control mode
 * @return true if payload is valid
 */
bool processSetGoalPayload(const PayloadSetGoal* payload, float* target_pos, uint16_t* duration_ms, uint8_t* mode);

/**
 * @brief Compute status flags based on current system state
 * @param is_moving Whether motor is moving
 * @param at_goal Whether motor is at goal position
 * @param error Whether an error occurred
 * @return Status flags byte
 */
uint8_t computeStatusFlags(bool is_moving, bool at_goal, bool error = false);

#endif // PROTOCOL_H
