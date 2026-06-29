#include <Arduino.h>
#include <Servo.h>
#include "config.h"
#include "X42S_CAN.h"

// ===== 全局对象 =====
X42S_Motor* motors[MOTOR_COUNT];
Servo gripperServo;

// 电位器引脚数组
const int potPins[POT_COUNT] = POT_PINS;

// 滑动平均滤波
#define FILTER_WINDOW 32
int potFilter[POT_COUNT][FILTER_WINDOW];
int potIndex[POT_COUNT] = {0};
bool potReady[POT_COUNT] = {false};

// 目标角度（输出轴角度，单位：0.1°）
int32_t targetAngles[MOTOR_COUNT];

// ===== 函数声明 =====
void initMotors();
void readPotentiometers();
void updateMotors();
void updateServo();
void collectAndSendData();
void printStatus();

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== LeRobot 6-Axis Arm (TWAI) ===");
    
    // 初始化 TWAI，传入波特率（来自 config.h）
    if (!X42S_Motor::initTWAI(CAN_BAUDRATE)) {
        Serial.println("TWAI init failed!");
        while (1) delay(1000);
    }

    // 初始化 TWAI
    if (!X42S_Motor::initTWAI(CAN_BAUDRATE)) {
        Serial.println("TWAI init failed!");
        while (1) delay(1000);
    }
    
    // 创建电机对象
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i] = new X42S_Motor(MOTOR_ID_START + i);
        Serial.printf("Motor %d ID=%d\n", i+1, MOTOR_ID_START + i);
    }
    
    // 使能所有电机
    Serial.print("Enabling motors... ");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!motors[i]->enable(true)) {
            Serial.printf("Motor %d enable failed\n", i+1);
        }
        delay(20);
    }
    Serial.println("OK");
    
    // 初始化电位器滤波
    for (int i = 0; i < POT_COUNT; i++) {
        for (int j = 0; j < FILTER_WINDOW; j++) potFilter[i][j] = 0;
        potIndex[i] = 0;
        potReady[i] = false;
    }
    
    // 舵机
    gripperServo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    gripperServo.write(90);
    
    analogReadResolution(12);
    Serial.println("=== Ready ===");
}

// ===== Loop =====
int32_t lastSentAngles[MOTOR_COUNT] = {0};   // 记录上次发送的角度

void loop() {
    static unsigned long lastDataSend = 0;
    unsigned long now = millis();
    
    readPotentiometers();   // 读取电位器，更新 targetAngles
    
    bool needUpdate = false;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (abs(targetAngles[i] - lastSentAngles[i]) >= 6) {  //  死区
            needUpdate = true;
            lastSentAngles[i] = targetAngles[i];
        }
    }
    
    if (needUpdate) {
        updateMotors();    // 发送命令
    }
    
    updateServo();         // 舵机控制
    
    if (ENABLE_DATA_COLLECTION && (now - lastDataSend >= DATA_SEND_INTERVAL)) {
        lastDataSend = now;
        collectAndSendData();
    }
    
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 5000) {
        lastPrint = now;
        printStatus();
    }
    
    delay(5);
}

// ===== 读取电位器  =====
void readPotentiometers() {
    for (int i = 0; i < POT_COUNT; i++) {
        int raw = analogRead(potPins[i]);
        potFilter[i][potIndex[i]] = raw;
        potIndex[i] = (potIndex[i] + 1) % FILTER_WINDOW;
        
        if (!potReady[i]) {
            if (potIndex[i] == 0) potReady[i] = true;
            continue;
        }
        
        long sum = 0;
        for (int j = 0; j < FILTER_WINDOW; j++) sum += potFilter[i][j];
        int filtered = sum / FILTER_WINDOW;
        
        // 映射到 0~3600 (0.1°)
        int32_t angle = map(filtered, 0, 4095, POSITION_MIN, POSITION_MAX);
        if (angle < 50) angle = 0;
        if (angle > 3550) angle = 3600;
        
        // 前6个电位器6个关节，7给舵机
        if (i < MOTOR_COUNT) {
            targetAngles[i] = angle;
        }
    }
}

// ===== 更新电机（带减速比修正） =====
void updateMotors() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // 目标输出轴角度乘以减速比 = 电机转子角度
        int32_t motorAngle = (int32_t)(targetAngles[i] * GEAR_RATIO);
        
        // 使用梯形加减速模式
        motors[i]->moveToTrap(motorAngle, DEFAULT_SPEED, DEFAULT_ACCEL, DEFAULT_ACCEL, 0x01, false);
    }
}

// ===== 更新舵机（7） =====
void updateServo() {
    int raw = analogRead(potPins[6]);
    int angle = map(raw, 0, 4095, 0, 180);
    gripperServo.write(angle);
}

// ===== 数据采集（LeRobot） =====
void collectAndSendData() {
    String json = "{";
    json += "\"timestamp\":" + String(millis()) + ",";
    
    // 电位器原始值 (指令)
    json += "\"cmd\":[";
    for (int i = 0; i < POT_COUNT; i++) {
        int raw = analogRead(potPins[i]);
        json += String(raw);
        if (i < POT_COUNT - 1) json += ",";
    }
    json += "],";
    
    // 当前目标输出轴角度 (状态)
    json += "\"state\":[";
    for (int i = 0; i < MOTOR_COUNT; i++) {
        json += String(targetAngles[i] / 10.0, 1);
        if (i < MOTOR_COUNT - 1) json += ",";
    }
    json += "],";
    
    json += "\"gripper\":" + String(gripperServo.read());
    json += "}";
    
    Serial.println(json);
}

// ===== 状态打印 =====
void printStatus() {
    Serial.println("=== Motor Targets ===");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Serial.printf("M%d: %.1f°\n", i+1, targetAngles[i] / 10.0);
    }
    Serial.println();
}