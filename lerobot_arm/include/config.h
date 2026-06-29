#ifndef CONFIG_H
#define CONFIG_H

//  CAN 配置 
#define CAN_BAUDRATE 500000      // 500Kbps

// 电机
#define MOTOR_COUNT 6            // 电机总数
#define MOTOR_ID_START 1         // 起始 ID
#define GEAR_RATIO 50.0f          // 减速比

//   电位器引脚 
#define POT_COUNT 7              // 电位器数
#define POT_PINS {36, 39, 34, 35, 32, 33, 25}  //   ESP32引脚

//   舵机配置（夹爪）   
#define SERVO_PIN 25
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

// 运动参数                                     
#define DEFAULT_SPEED 32000       // 速度  0.1 RPM
#define DEFAULT_ACCEL 2200      // 加速度 RPM/s）
#define POSITION_MIN 1           // 位置范围（0.1°）
#define POSITION_MAX 3600        // 0 ~ 360.0°

// 数据采集 
#define ENABLE_DATA_COLLECTION true
#define DATA_SEND_INTERVAL 10    // 发送间隔 (ms)

#endif