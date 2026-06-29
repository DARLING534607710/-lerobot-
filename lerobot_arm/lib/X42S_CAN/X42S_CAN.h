#ifndef X42S_CAN_H
#define X42S_CAN_H

#include <Arduino.h>

#define CHECKSUM 0x6B

// X固件命令码
#define CMD_MOTOR_ENABLE       0xF3
#define CMD_POSITION_DIRECT    0xFB
#define CMD_POSITION_TRAP      0xFD
#define CMD_SPEED_MODE         0xF6
#define CMD_IMMEDIATE_STOP     0xFE
#define CMD_SYNC_START         0xFF

// 状态标志
#define FLAG_ENABLE            0x01
#define FLAG_POSITION_REACHED  0x02

class X42S_Motor {
public:
    X42S_Motor(uint8_t id);
    
    // 初始化 TWAI
    static bool initTWAI(int baudrate);
    
    bool enable(bool on, bool sync = false);
    bool moveToDirect(int32_t position, uint16_t speed, uint8_t mode = 0x01, bool sync = false);
    bool moveToTrap(int32_t position, uint16_t speed, uint16_t accel, uint16_t decel,
                    uint8_t mode = 0x01, bool sync = false);
    bool stop(bool sync = false);
    uint8_t getId() { return _id; }
    
private:
    uint8_t _id;
    bool sendCommand(const uint8_t* data, size_t len, int timeout_ms = 100);
};

#endif