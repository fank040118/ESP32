/*
#include <Arduino.h>

// MX1508电机驱动器引脚定义
// 连接到电机A (Motor A)
#define MOTOR_IN1 33    // MX1508的IN1引脚
#define MOTOR_IN2 32    // MX1508的IN2引脚

// PWM参数设置 - 用于限制启动电流
#define PWM_FREQ 1000   // PWM频率
#define PWM_RESOLUTION 8 // PWM分辨率(8位，0-255)
#define PWM_CHANNEL_1 0  // PWM通道1
#define PWM_CHANNEL_2 1  // PWM通道2

void setup() {
  // 初始化串口监视器
  Serial.begin(115200);
  Serial.println("TT电机测试开始!");
  
  // 配置PWM通道 - 用于软启动减少电流冲击
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  
  // 将PWM通道绑定到引脚
  ledcAttachPin(MOTOR_IN1, PWM_CHANNEL_1);
  ledcAttachPin(MOTOR_IN2, PWM_CHANNEL_2);
  
  // 初始状态：电机停止
  ledcWrite(PWM_CHANNEL_1, 0);
  ledcWrite(PWM_CHANNEL_2, 0);
  
  Serial.println("电机已初始化(带软启动保护)，准备开始测试...");
  delay(2000);
}

// 软启动电机正转函数 - 避免大电流冲击
void motorForward() {
  Serial.println("  -> 软启动正转");
  // 渐进启动，从30%功率开始，避免大电流
  for(int speed = 80; speed <= 200; speed += 20) {
    ledcWrite(PWM_CHANNEL_1, speed);
    ledcWrite(PWM_CHANNEL_2, 0);
    delay(100);  // 每步延时100ms
  }
}

// 软启动电机反转函数
void motorBackward() {
  Serial.println("  -> 软启动反转");
  // 渐进启动，从30%功率开始
  for(int speed = 80; speed <= 200; speed += 20) {
    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, speed);
    delay(100);
  }
}

// 电机停止函数
void motorStop() {
  ledcWrite(PWM_CHANNEL_1, 0);
  ledcWrite(PWM_CHANNEL_2, 0);
}

// 快速模式(较低功率，适合无负载测试)
void motorForwardQuick() {
  ledcWrite(PWM_CHANNEL_1, 150);  // 约60%功率
  ledcWrite(PWM_CHANNEL_2, 0);
}

void loop() {
  // 测试1：正转
  Serial.println("测试1: 电机正转");
  motorForward();
  delay(3000);  // 正转3秒
  
  // 停止
  Serial.println("停止");
  motorStop();
  delay(1000);  // 停止1秒
  
  // 测试2：反转
  Serial.println("测试2: 电机反转");
  motorBackward();
  delay(3000);  // 反转3秒
  
  // 停止
  Serial.println("停止");
  motorStop();
  delay(1000);  // 停止1秒
  
  // 测试3：快速启停测试(使用较低功率)
  Serial.println("测试3: 快速启停测试(低功率模式)");
  for(int i = 0; i < 5; i++) {
    Serial.print("快速正转 ");
    Serial.println(i+1);
    motorForwardQuick();  // 使用快速低功率模式
    delay(500);
    motorStop();
    delay(500);
  }
  
  Serial.println("一轮测试完成，等待5秒后重复...");
  delay(5000);
}
*/

#include <Arduino.h>

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

// 全局变量来控制程序只运行一次
bool testCompleted = false;

// 双电机停止函数
void motorsStop() {
  ledcWrite(PWM_CHANNEL_M1_1, 0);
  ledcWrite(PWM_CHANNEL_M1_2, 0);
  ledcWrite(PWM_CHANNEL_M2_1, 0);
  ledcWrite(PWM_CHANNEL_M2_2, 0);
}

// 双电机同步正转函数 - 软启动避免大电流
void motorsForward() {
  Serial.println("  -> 双电机软启动同步正转");
  // 渐进启动，从80开始，避免大电流
  for(int speed = 80; speed <= 200; speed += 20) {
    // 电机1正转
    ledcWrite(PWM_CHANNEL_M1_1, speed);
    ledcWrite(PWM_CHANNEL_M1_2, 0);
    // 电机2正转
    ledcWrite(PWM_CHANNEL_M2_1, speed);
    ledcWrite(PWM_CHANNEL_M2_2, 0);
    delay(100);  // 每步延时100ms
  }
}

// 双电机同步反转函数
void motorsBackward() {
  Serial.println("  -> 双电机软启动同步反转");
  // 渐进启动，从80开始
  for(int speed = 80; speed <= 200; speed += 20) {
    // 电机1反转
    ledcWrite(PWM_CHANNEL_M1_1, 0);
    ledcWrite(PWM_CHANNEL_M1_2, speed);
    // 电机2反转
    ledcWrite(PWM_CHANNEL_M2_1, 0);
    ledcWrite(PWM_CHANNEL_M2_2, speed);
    delay(100);
  }
}

// 双电机反向转动函数(电机1正转，电机2反转)
void motorsOpposite() {
  Serial.println("  -> 双电机反向转动(电机1正转，电机2反转)");
  for(int speed = 80; speed <= 200; speed += 20) {
    // 电机1正转
    ledcWrite(PWM_CHANNEL_M1_1, speed);
    ledcWrite(PWM_CHANNEL_M1_2, 0);
    // 电机2反转
    ledcWrite(PWM_CHANNEL_M2_1, 0);
    ledcWrite(PWM_CHANNEL_M2_2, speed);
    delay(100);
  }
}

// 快速模式(较低功率，适合无负载测试)
void motorsForwardQuick() {
  // 电机1快速正转
  ledcWrite(PWM_CHANNEL_M1_1, 150);  // 约60%功率
  ledcWrite(PWM_CHANNEL_M1_2, 0);
  // 电机2快速正转
  ledcWrite(PWM_CHANNEL_M2_1, 150);
  ledcWrite(PWM_CHANNEL_M2_2, 0);
}

void setup() {
  // 初始化串口监视器
  Serial.begin(115200);
  Serial.println("双TT电机同步测试开始!");
  
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
  
  Serial.println("双电机已初始化(带软启动保护)，准备开始测试...");
  delay(2000);
}

void loop() {
  // 检查测试是否已完成
  if (testCompleted) {
    return;  // 如果测试完成，退出loop函数
  }
  
  Serial.println("=== 开始双电机同步测试 ===");
  
  // 测试1：双电机同步正转
  Serial.println("测试1: 双电机同步正转");
  motorsForward();
  delay(3000);  // 正转3秒
  
  // 停止
  Serial.println("停止");
  motorsStop();
  delay(1000);  // 停止1秒
  
  // 测试2：双电机同步反转
  Serial.println("测试2: 双电机同步反转");
  motorsBackward();
  delay(3000);  // 反转3秒
  
  // 停止
  Serial.println("停止");
  motorsStop();
  delay(1000);  // 停止1秒
  
  // 测试3：双电机反向转动
  Serial.println("测试3: 双电机反向转动");
  motorsOpposite();
  delay(3000);  // 反向转动3秒
  
  // 停止
  Serial.println("停止");
  motorsStop();
  delay(1000);  // 停止1秒
  
  // 测试4：快速启停测试(使用较低功率)
  Serial.println("测试4: 双电机快速启停测试(低功率模式)");
  for(int i = 0; i < 5; i++) {
    Serial.print("快速正转 ");
    Serial.println(i+1);
    motorsForwardQuick();  // 使用快速低功率模式
    delay(500);
    motorsStop();
    delay(500);
  }
  
  Serial.println("=== 双电机测试完成！===");
  Serial.println("程序结束，不会重复执行。");
  
  // 标记测试完成
  testCompleted = true;
}
