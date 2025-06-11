#include <Arduino.h>
#include "BluetoothSerial.h"

// 电机1引脚定义 (Motor 1)
#define MOTOR1_IN1 32    // 电机1的IN1引脚
#define MOTOR1_IN2 33    // 电机1的IN2引脚

// 电机2引脚定义 (Motor 2)  
#define MOTOR2_IN1 25    // 电机2的IN1引脚
#define MOTOR2_IN2 26    // 电机2的IN2引脚

// PWM参数设置 - 用于限制启动电流
#define PWM_FREQ 1000   // PWM频率(PWM Frequency)
#define PWM_RESOLUTION 8 // PWM分辨率(PWM Resolution)，8位，0-255

// PWM通道分配(PWM Channel Assignment)
#define PWM_CHANNEL_M1_1 0  // 电机1-IN1的PWM通道
#define PWM_CHANNEL_M1_2 1  // 电机1-IN2的PWM通道
#define PWM_CHANNEL_M2_1 2  // 电机2-IN1的PWM通道
#define PWM_CHANNEL_M2_2 3  // 电机2-IN2的PWM通道

// 创建蓝牙串口对象
BluetoothSerial SerialBT;

// 电机控制函数声明
void motorsStop();
void motorsForward();
void motorsBackward();

void setup() {
  // 初始化串口监视器
  Serial.begin(115200);

  // 初始化蓝牙
  SerialBT.begin("ESP32_Motor_Control");
  Serial.println("蓝牙已启动，设备名称：ESP32_Motor_Control");
  
  // 配置所有PWM通道 - 用于软启动减少电流冲击
  ledcSetup(PWM_CHANNEL_M1_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_M1_2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_M2_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_M2_2, PWM_FREQ, PWM_RESOLUTION);
  
  // 将PWM通道绑定到引脚
  ledcAttachPin(MOTOR1_IN1, PWM_CHANNEL_M1_1);
  ledcAttachPin(MOTOR1_IN2, PWM_CHANNEL_M1_2);
  ledcAttachPin(MOTOR2_IN1, PWM_CHANNEL_M2_1);
  ledcAttachPin(MOTOR2_IN2, PWM_CHANNEL_M2_2);
  
  // 初始状态：所有电机停止
  motorsStop();
  
  Serial.println("系统初始化完成");
}

// 双电机停止函数
void motorsStop() {
  ledcWrite(PWM_CHANNEL_M1_1, 0);
  ledcWrite(PWM_CHANNEL_M1_2, 0);
  ledcWrite(PWM_CHANNEL_M2_1, 0);
  ledcWrite(PWM_CHANNEL_M2_2, 0);
}

// 双电机同步正转函数
void motorsForward() {
  // 电机1正转
  ledcWrite(PWM_CHANNEL_M1_1, 150);
  ledcWrite(PWM_CHANNEL_M1_2, 0);
  // 电机2正转
  ledcWrite(PWM_CHANNEL_M2_1, 150);
  ledcWrite(PWM_CHANNEL_M2_2, 0);
}

// 双电机同步反转函数
void motorsBackward() {
  // 电机1反转
  ledcWrite(PWM_CHANNEL_M1_1, 0);
  ledcWrite(PWM_CHANNEL_M1_2, 150);
  // 电机2反转
  ledcWrite(PWM_CHANNEL_M2_1, 0);
  ledcWrite(PWM_CHANNEL_M2_2, 150);
}

void loop() {
  delay(10);  // 小延迟以稳定程序执行
}



