/**
 * @file    protocol.cpp
 * @brief   Implementation of High-Speed Binary Communication Protocol
 * @details CRC calculation, packet parsing, and validation functions
 * @author  M-TRCH
 * @date    2025-12-03
 * @version 1.0
 */

#include "protocol.h"

// Global variables
static bool protocol_initialized = false;

/**
 * @brief Initialize the protocol system
 */
void protocolInit() {
    protocol_initialized = true;
}

/**
 * @brief Update CRC-16-IBM with a single byte
 * @details Uses polynomial 0xA001 (reversed 0x8005)
 * @param crc Current CRC value
 * @param data Byte to add to CRC
 * @return Updated CRC value
 */
uint16_t crc16Update(uint16_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 1) {
            crc = (crc >> 1) ^ 0xA001;  // CRC-16-IBM polynomial
        } else {
            crc >>= 1;
        }
    }
    return crc;
}

/**
 * @brief Calculate CRC-16-IBM for data buffer
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @return CRC-16 value
 */
uint16_t calculateCRC16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;  // Initial value
    
    for (uint16_t i = 0; i < length; i++) {
        crc = crc16Update(crc, data[i]);
    }
    
    return crc;
}

/**
 * @brief Verify CRC of a received packet
 * @param pkt Pointer to BinaryPacket structure
 * @return true if CRC is valid, false otherwise
 */
bool verifyCRC16(const BinaryPacket* pkt) {
    if (pkt == nullptr) return false;
    
    // Calculate CRC over: packet_type, payload_len, payload
    uint16_t calculated_crc = 0xFFFF;
    calculated_crc = crc16Update(calculated_crc, pkt->packet_type);
    calculated_crc = crc16Update(calculated_crc, pkt->payload_len);
    
    for (uint8_t i = 0; i < pkt->payload_len; i++) {
        calculated_crc = crc16Update(calculated_crc, pkt->payload[i]);
    }
    
    // Compare with received CRC (little-endian)
    return (calculated_crc == pkt->crc16);
}

/**
 * @brief Calculate and set CRC for a packet before sending
 * @param pkt Pointer to BinaryPacket structure
 */
void setCRC16(BinaryPacket* pkt) {
    if (pkt == nullptr) return;
    
    // Calculate CRC over: packet_type, payload_len, payload
    uint16_t crc = 0xFFFF;
    crc = crc16Update(crc, pkt->packet_type);
    crc = crc16Update(crc, pkt->payload_len);
    
    for (uint8_t i = 0; i < pkt->payload_len; i++) {
        crc = crc16Update(crc, pkt->payload[i]);
    }
    
    pkt->crc16 = crc;  // Store as little-endian
}

/**
 * @brief Check if binary packet is available on serial port
 * @param serial Pointer to HardwareSerial object
 * @return true if binary packet header detected, false otherwise
 */
bool isBinaryPacketAvailable(HardwareSerial* serial) {
    if (serial == nullptr || serial->available() < 2) return false;
    
    // Peek at first byte
    uint8_t first_byte = serial->peek();
    return (first_byte == PROTOCOL_HEADER_1);
}

/**
 * @brief Receive and parse a binary packet from serial port
 * @param serial Pointer to HardwareSerial object
 * @param pkt Pointer to BinaryPacket structure to fill
 * @param timeout_ms Timeout in milliseconds
 * @return true if packet received and CRC valid, false otherwise
 */
bool receivePacket(HardwareSerial* serial, BinaryPacket* pkt, uint32_t timeout_ms) {
    if (serial == nullptr || pkt == nullptr) return false;
    
    uint32_t start_time = millis();
    
    // Wait for header bytes
    while (serial->available() < 2) {
        if (millis() - start_time > timeout_ms) return false;
    }
    
    // Read and verify header
    if (serial->read() != PROTOCOL_HEADER_1) return false;
    if (serial->read() != PROTOCOL_HEADER_2) return false;
    
    pkt->header[0] = PROTOCOL_HEADER_1;
    pkt->header[1] = PROTOCOL_HEADER_2;
    
    // Wait for packet_type and payload_len
    start_time = millis();
    while (serial->available() < 2) {
        if (millis() - start_time > timeout_ms) return false;
    }
    
    pkt->packet_type = serial->read();
    pkt->payload_len = serial->read();
    
    // Validate payload length
    if (pkt->payload_len > PROTOCOL_MAX_PAYLOAD_SIZE) return false;
    
    // Wait for payload
    start_time = millis();
    while (serial->available() < pkt->payload_len) {
        if (millis() - start_time > timeout_ms) return false;
    }
    
    // Read payload
    for (uint8_t i = 0; i < pkt->payload_len; i++) {
        pkt->payload[i] = serial->read();
    }
    
    // Wait for CRC bytes (2 bytes, little-endian)
    start_time = millis();
    while (serial->available() < 2) {
        if (millis() - start_time > timeout_ms) return false;
    }
    
    // Read CRC (little-endian)
    uint8_t crc_low = serial->read();
    uint8_t crc_high = serial->read();
    pkt->crc16 = (uint16_t)crc_low | ((uint16_t)crc_high << 8);
    
    // Verify CRC
    return verifyCRC16(pkt);
}

/**
 * @brief Send a binary packet via serial port
 * @param serial Pointer to HardwareSerial object
 * @param pkt Pointer to BinaryPacket structure to send
 */
void sendPacket(HardwareSerial* serial, const BinaryPacket* pkt) {
    if (serial == nullptr || pkt == nullptr) return;
    
    // Send header
    serial->write(pkt->header[0]);
    serial->write(pkt->header[1]);
    
    // Send packet type and payload length
    serial->write(pkt->packet_type);
    serial->write(pkt->payload_len);
    
    // Send payload
    for (uint8_t i = 0; i < pkt->payload_len; i++) {
        serial->write(pkt->payload[i]);
    }
    
    // Send CRC (little-endian)
    serial->write((uint8_t)(pkt->crc16 & 0xFF));        // Low byte
    serial->write((uint8_t)((pkt->crc16 >> 8) & 0xFF)); // High byte
}

/**
 * @brief Send a status feedback packet
 * @param serial Pointer to HardwareSerial object
 * @param motor_id Motor ID
 * @param pos Current position (degrees * 100 or encoder ticks)
 * @param current Current in mA
 * @param flags Status flags
 */
void sendStatusFeedback(HardwareSerial* serial, uint8_t motor_id, int32_t pos, int16_t current, uint8_t flags) {
    BinaryPacket pkt;
    
    // Set header
    pkt.header[0] = PROTOCOL_HEADER_1;
    pkt.header[1] = PROTOCOL_HEADER_2;
    
    // Set packet type
    pkt.packet_type = PKT_FB_STATUS;
    
    // Build payload
    PayloadStatus* status = (PayloadStatus*)pkt.payload;
    status->motor_id = motor_id;
    status->actual_pos = pos;
    status->actual_current = current;
    status->status_flags = flags;
    
    pkt.payload_len = sizeof(PayloadStatus);
    
    // Calculate and set CRC
    setCRC16(&pkt);
    
    // Send packet
    sendPacket(serial, &pkt);
}

/**
 * @brief Send an error feedback packet
 * @param serial Pointer to HardwareSerial object
 * @param error_code Error code
 * @param last_pkt_type Packet type that caused error
 * @param debug_info Additional debug info
 */
void sendErrorFeedback(HardwareSerial* serial, uint8_t error_code, uint8_t last_pkt_type, uint16_t debug_info) {
    BinaryPacket pkt;
    
    // Set header
    pkt.header[0] = PROTOCOL_HEADER_1;
    pkt.header[1] = PROTOCOL_HEADER_2;
    
    // Set packet type
    pkt.packet_type = PKT_FB_ERROR;
    
    // Build payload
    PayloadError* error = (PayloadError*)pkt.payload;
    error->error_code = error_code;
    error->last_packet_type = last_pkt_type;
    error->debug_info = debug_info;
    
    pkt.payload_len = sizeof(PayloadError);
    
    // Calculate and set CRC
    setCRC16(&pkt);
    
    // Send packet
    sendPacket(serial, &pkt);
}

/**
 * @brief Process received CMD_SET_GOAL packet
 * @param payload Pointer to PayloadSetGoal structure
 * @param target_pos Output: target position (degrees)
 * @param duration_ms Output: duration (only for S-Curve mode)
 * @param mode Output: control mode
 * @return true if payload is valid
 */
bool processSetGoalPayload(const PayloadSetGoal* payload, float* target_pos, uint16_t* duration_ms, uint8_t* mode) {
    if (payload == nullptr || target_pos == nullptr || duration_ms == nullptr || mode == nullptr) {
        return false;
    }
    
    *mode = payload->control_mode;
    
    switch (payload->control_mode) {
        case MODE_DIRECT_POSITION:
            // Convert int32 (degrees * 100) to float degrees
            *target_pos = (float)payload->data.direct.target_pos / 100.0f;
            *duration_ms = 0;
            return true;
            
        case MODE_SCURVE_PROFILE:
            // Convert int32 (degrees * 100) to float degrees
            *target_pos = (float)payload->data.scurve.target_pos / 100.0f;
            *duration_ms = payload->data.scurve.duration_ms;
            return true;
            
        default:
            return false;  // Invalid mode
    }
}

/**
 * @brief Compute status flags based on current system state
 * @param is_moving Whether motor is moving
 * @param at_goal Whether motor is at goal position
 * @param error Whether an error occurred
 * @return Status flags byte
 */
uint8_t computeStatusFlags(bool is_moving, bool at_goal, bool error) {
    uint8_t flags = 0;
    
    if (is_moving) flags |= STATUS_MOVING;
    if (at_goal) flags |= STATUS_AT_GOAL;
    if (error) flags |= STATUS_ERROR;
    
    // TODO: Add additional status checks (overheat, overcurrent, encoder error)
    
    return flags;
}
