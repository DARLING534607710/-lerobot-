#include "X42S_CAN.h"
#include "driver/twai.h"

// ===== 静态初始化（执行一次） =====
bool X42S_Motor::initTWAI(int baudrate) {
    static bool initialized = false;
    if (initialized) {
        Serial.println("TWAI already initialized");
        return true;
    }

    twai_driver_uninstall();  // 清理

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)22, (gpio_num_t)23, TWAI_MODE_NORMAL
    );

    twai_timing_config_t t_config;
    switch (baudrate) {
        case 125000:  t_config = TWAI_TIMING_CONFIG_125KBITS(); break;
        case 250000:  t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
        case 500000:  t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
        default:      t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("TWAI install failed, err=%d (0x%x)\n", err, err);
        return false;
    }
    Serial.println("TWAI driver installed");

    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("TWAI start failed, err=%d (0x%x)\n", err, err);
        twai_driver_uninstall();
        return false;
    }
    Serial.println("TWAI started");

    initialized = true;
    return true;
}

// ===== 构造函数 =====
X42S_Motor::X42S_Motor(uint8_t id) : _id(id) {}

// ===== 底层发送 =====
bool X42S_Motor::sendCommand(const uint8_t* data, size_t len, int timeout_ms) {
    if (len == 0) return false;
    if (len <= 8) {
        twai_message_t msg;
        msg.extd = 1;
        msg.rtr = 0;
        msg.data_length_code = len;
        msg.identifier = (_id << 8) | 0;
        memcpy(msg.data, data, len);
        return twai_transmit(&msg, pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
    }
    // 分包：第一包发前 8 字节
    twai_message_t msg1;
    msg1.extd = 1;
    msg1.rtr = 0;
    msg1.data_length_code = 8;
    msg1.identifier = (_id << 8) | 0;
    memcpy(msg1.data, data, 8);
    if (twai_transmit(&msg1, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) return false;
    
    // 第二包：功能码 + 剩余数据
    size_t remaining = len - 8;
    size_t secondLen = 1 + remaining;
    twai_message_t msg2;
    msg2.extd = 1;
    msg2.rtr = 0;
    msg2.data_length_code = secondLen;
    msg2.identifier = (_id << 8) | 1;
    msg2.data[0] = data[0];
    memcpy(&msg2.data[1], &data[8], remaining);
    return twai_transmit(&msg2, pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
}

// ===== 使能 =====
bool X42S_Motor::enable(bool on, bool sync) {
    uint8_t cmd[] = { CMD_MOTOR_ENABLE, 0xAB, (uint8_t)(on ? 1 : 0), (uint8_t)(sync ? 1 : 0), CHECKSUM };
    return sendCommand(cmd, sizeof(cmd));
}

// ===== 直通位置 =====
bool X42S_Motor::moveToDirect(int32_t position, uint16_t speed, uint8_t mode, bool sync) {
    uint8_t dir = (position >= 0) ? 0x01 : 0x00;
    uint32_t absPos = (position >= 0) ? (uint32_t)position : (uint32_t)(-position);
    uint8_t cmd[] = {
        CMD_POSITION_DIRECT, dir,
        (uint8_t)(speed >> 8), (uint8_t)(speed & 0xFF),
        (uint8_t)(absPos >> 24), (uint8_t)(absPos >> 16),
        (uint8_t)(absPos >> 8), (uint8_t)(absPos & 0xFF),
        mode,
        (uint8_t)(sync ? 1 : 0),
        CHECKSUM
    };
    return sendCommand(cmd, sizeof(cmd));
}

// ===== 梯形加减速位置 =====
bool X42S_Motor::moveToTrap(int32_t position, uint16_t speed, uint16_t accel, uint16_t decel,
                            uint8_t mode, bool sync) {
    uint8_t dir = (position >= 0) ? 0x01 : 0x00;
    uint32_t absPos = (position >= 0) ? (uint32_t)position : (uint32_t)(-position);
    uint8_t cmd[] = {
        CMD_POSITION_TRAP, dir,
        (uint8_t)(accel >> 8), (uint8_t)(accel & 0xFF),
        (uint8_t)(decel >> 8), (uint8_t)(decel & 0xFF),
        (uint8_t)(speed >> 8), (uint8_t)(speed & 0xFF),
        (uint8_t)(absPos >> 24), (uint8_t)(absPos >> 16),
        (uint8_t)(absPos >> 8), (uint8_t)(absPos & 0xFF),
        mode,
        (uint8_t)(sync ? 1 : 0),
        CHECKSUM
    };
    return sendCommand(cmd, sizeof(cmd));
}

// ===== 立即停止 =====
bool X42S_Motor::stop(bool sync) {
    uint8_t cmd[] = { CMD_IMMEDIATE_STOP, 0x98, (uint8_t)(sync ? 1 : 0), CHECKSUM };
    return sendCommand(cmd, sizeof(cmd));
}