#include <Arduino.h>
#include "BluetoothSerial.h"

// 电机1引脚定义 (右电机 - Right Motor)
#define MOTOR_RIGHT_IN1 32    // 右电机的IN1引脚
#define MOTOR_RIGHT_IN2 33    // 右电机的IN2引脚

// 电机2引脚定义 (左电机 - Left Motor)  
#define MOTOR_LEFT_IN1 25     // 左电机的IN1引脚
#define MOTOR_LEFT_IN2 26     // 左电机的IN2引脚

// 超声波模块引脚定义
#define TRIG_PIN 27           // 超声波模块的Trig引脚
#define ECHO_PIN 14           // 超声波模块的Echo引脚

// LED和按钮引脚定义
#define LED_PIN 22            // LED连接到GPIO22
#define BUTTON_PIN 23         // 按钮连接到GPIO23

// PWM参数设置
#define PWM_FREQ 1000         // PWM频率
#define PWM_RESOLUTION 8      // PWM分辨率，8位，0-255

// PWM通道分配
#define PWM_CHANNEL_RIGHT_1 0  // 右电机-IN1的PWM通道
#define PWM_CHANNEL_RIGHT_2 1  // 右电机-IN2的PWM通道
#define PWM_CHANNEL_LEFT_1 2   // 左电机-IN1的PWM通道
#define PWM_CHANNEL_LEFT_2 3   // 左电机-IN2的PWM通道

// 超声波参数
#define MAX_DISTANCE 400      // 最大测距范围(cm)
#define TIMEOUT_US 30000      // 超时时间(微秒)
#define OBSTACLE_THRESHOLD 5  // 障碍物检测阈值(cm)

// 创建蓝牙串口对象
BluetoothSerial SerialBT;

// 按钮防抖变量
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// 测试状态变量
bool testRunning = false;

// 超声波检测变量
bool obstacleDetected = false;
unsigned long lastDistanceCheck = 0;
unsigned long distanceCheckInterval = 100; // 每100ms检测一次距离

// 软启动参数
int maxMotorSpeed = 180;      // 最大速度调整为70% (255 * 0.7 = 178)
int currentSpeed = 0;         // 当前速度
int minSpeed = 30;            // 最小启动速度
int speedStep = 10;           // 每次速度递增值
int speedDelay = 50;          // 速度递增间隔(ms)

// 电机状态枚举
enum MotorState {
  STOPPED,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
};

MotorState currentMotorState = STOPPED;
MotorState targetMotorState = STOPPED;

// 函数声明
void motorsStop();
void motorsForward();
void motorsBackward();
void motorsLeft();
void motorsRight();
void runMotorTest();
bool readButtonWithDebounce();
void processBluetoothCommand();
void updateMotorSpeed();
void setMotorDirection(MotorState direction);
float measureDistance();
void checkObstacle();

void setup() {
  // 初始化串口监视器
  Serial.begin(115200);

  // 初始化蓝牙
  SerialBT.begin("ESP32_Motor_Control");
  Serial.println("蓝牙已启动，设备名称：ESP32_Motor_Control");
  Serial.println("蓝牙控制命令：");
  Serial.println("F = 前进, B = 后退, L = 左转, R = 右转, S = 停止");
  Serial.print("最大电机速度设置为: ");
  Serial.print(maxMotorSpeed);
  Serial.println(" (70% 功率)");
  Serial.println("障碍物检测已启用，阈值: 5cm");
  
  // 配置按钮和LED引脚
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // 配置超声波引脚
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  
  // 配置所有PWM通道
  ledcSetup(PWM_CHANNEL_RIGHT_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT_2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LEFT_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LEFT_2, PWM_FREQ, PWM_RESOLUTION);
  
  // 将PWM通道绑定到引脚
  ledcAttachPin(MOTOR_RIGHT_IN1, PWM_CHANNEL_RIGHT_1);
  ledcAttachPin(MOTOR_RIGHT_IN2, PWM_CHANNEL_RIGHT_2);
  ledcAttachPin(MOTOR_LEFT_IN1, PWM_CHANNEL_LEFT_1);
  ledcAttachPin(MOTOR_LEFT_IN2, PWM_CHANNEL_LEFT_2);
  
  // 初始状态：所有电机停止
  motorsStop();
  
  Serial.println("系统初始化完成");
  Serial.println("按下按钮开始电机测试，或使用蓝牙控制...");
}

// 测量距离函数
float measureDistance() {
  // 确保Trig引脚为低电平
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // 发送10微秒高电平脉冲触发测距
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 测量Echo引脚高电平持续时间
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
  
  // 如果超时，返回-1表示测距失败
  if (duration == 0) {
    return -1;
  }
  
  // 计算距离(cm)
  float distance = (duration * 0.034) / 2;
  
  return distance;
}

// 检查障碍物函数
void checkObstacle() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastDistanceCheck >= distanceCheckInterval) {
    lastDistanceCheck = currentTime;
    
    float distance = measureDistance();
    
    if (distance > 0 && distance <= OBSTACLE_THRESHOLD) {
      if (!obstacleDetected) {
        obstacleDetected = true;
        digitalWrite(LED_PIN, HIGH);
        Serial.print("检测到障碍物！距离: ");
        Serial.print(distance, 1);
        Serial.println(" cm");
        SerialBT.println("Obstacle detected! Forward/Turn commands blocked");
        
        // 如果当前正在前进或转向，立即停止
        if (targetMotorState == FORWARD || targetMotorState == LEFT || targetMotorState == RIGHT) {
          motorsStop();
          Serial.println("因检测到障碍物，电机已停止");
          SerialBT.println("Motors stopped due to obstacle");
        }
      }
    } else if (distance > OBSTACLE_THRESHOLD || distance == -1) {
      if (obstacleDetected) {
        obstacleDetected = false;
        digitalWrite(LED_PIN, LOW);
        if (distance > 0) {
          Serial.print("障碍物已移除，距离: ");
          Serial.print(distance, 1);
          Serial.println(" cm");
        } else {
          Serial.println("测距信号丢失，假设无障碍物");
        }
        SerialBT.println("Obstacle cleared! All commands available");
      }
    }
  }
}

// 设置电机方向（不改变速度）
void setMotorDirection(MotorState direction) {
  switch(direction) {
    case FORWARD:
      // 右电机正转方向
      ledcWrite(PWM_CHANNEL_RIGHT_2, 0);
      // 左电机正转方向
      ledcWrite(PWM_CHANNEL_LEFT_2, 0);
      break;
      
    case BACKWARD:
      // 右电机反转方向
      ledcWrite(PWM_CHANNEL_RIGHT_1, 0);
      // 左电机反转方向
      ledcWrite(PWM_CHANNEL_LEFT_1, 0);
      break;
      
    case LEFT:
      // 右电机正转，左电机反转
      ledcWrite(PWM_CHANNEL_RIGHT_2, 0);
      ledcWrite(PWM_CHANNEL_LEFT_1, 0);
      break;
      
    case RIGHT:
      // 右电机反转，左电机正转
      ledcWrite(PWM_CHANNEL_RIGHT_1, 0);
      ledcWrite(PWM_CHANNEL_LEFT_2, 0);
      break;
      
    case STOPPED:
    default:
      ledcWrite(PWM_CHANNEL_RIGHT_1, 0);
      ledcWrite(PWM_CHANNEL_RIGHT_2, 0);
      ledcWrite(PWM_CHANNEL_LEFT_1, 0);
      ledcWrite(PWM_CHANNEL_LEFT_2, 0);
      break;
  }
}

// 更新电机速度（软启动/软停止）
void updateMotorSpeed() {
  static unsigned long lastSpeedUpdate = 0;
  
  if (millis() - lastSpeedUpdate < speedDelay) {
    return; // 还没到更新时间
  }
  
  lastSpeedUpdate = millis();
  
  if (targetMotorState == STOPPED) {
    // 软停止：逐渐减速
    if (currentSpeed > 0) {
      currentSpeed -= speedStep * 2; // 停止时减速更快
      if (currentSpeed < 0) currentSpeed = 0;
      
      if (currentSpeed == 0) {
        currentMotorState = STOPPED;
        setMotorDirection(STOPPED);
        Serial.println("电机已停止");
        SerialBT.println("Motors STOPPED");
      }
    }
  } else {
    // 软启动：逐渐加速
    if (currentMotorState != targetMotorState) {
      // 改变方向时先停止
      if (currentSpeed > 0) {
        currentSpeed -= speedStep * 2;
        if (currentSpeed <= 0) {
          currentSpeed = 0;
          currentMotorState = targetMotorState;
          setMotorDirection(targetMotorState);
        }
      } else {
        currentMotorState = targetMotorState;
        setMotorDirection(targetMotorState);
      }
    } else {
      // 相同方向，逐渐加速
      if (currentSpeed < maxMotorSpeed) {
        if (currentSpeed == 0) {
          currentSpeed = minSpeed; // 从最小速度开始
        } else {
          currentSpeed += speedStep;
        }
        if (currentSpeed > maxMotorSpeed) {
          currentSpeed = maxMotorSpeed;
        }
      }
    }
  }
  
  // 应用当前速度
  if (currentSpeed > 0 && currentMotorState != STOPPED) {
    switch(currentMotorState) {
      case FORWARD:
        ledcWrite(PWM_CHANNEL_RIGHT_1, currentSpeed);
        ledcWrite(PWM_CHANNEL_LEFT_1, currentSpeed);
        break;
        
      case BACKWARD:
        ledcWrite(PWM_CHANNEL_RIGHT_2, currentSpeed);
        ledcWrite(PWM_CHANNEL_LEFT_2, currentSpeed);
        break;
        
      case LEFT:
        ledcWrite(PWM_CHANNEL_RIGHT_1, currentSpeed);
        ledcWrite(PWM_CHANNEL_LEFT_2, currentSpeed);
        break;
        
      case RIGHT:
        ledcWrite(PWM_CHANNEL_RIGHT_2, currentSpeed);
        ledcWrite(PWM_CHANNEL_LEFT_1, currentSpeed);
        break;
    }
  }
}

// 双电机停止函数
void motorsStop() {
  targetMotorState = STOPPED;
  Serial.println("正在停止电机...");
  SerialBT.println("Stopping motors...");
}

// 双电机前进函数 - 添加障碍物检测
void motorsForward() {
  if (obstacleDetected) {
    Serial.println("检测到障碍物，拒绝前进命令");
    SerialBT.println("Obstacle detected, FORWARD command blocked");
    return;
  }
  targetMotorState = FORWARD;
  Serial.println("电机前进");
  SerialBT.println("Motors FORWARD");
}

// 双电机后退函数 - 后退不受障碍物限制
void motorsBackward() {
  targetMotorState = BACKWARD;
  Serial.println("电机后退");
  SerialBT.println("Motors BACKWARD");
}

// 左转函数 - 添加障碍物检测
void motorsLeft() {
  if (obstacleDetected) {
    Serial.println("检测到障碍物，拒绝左转命令");
    SerialBT.println("Obstacle detected, LEFT command blocked");
    return;
  }
  targetMotorState = LEFT;
  Serial.println("左转");
  SerialBT.println("Turn LEFT");
}

// 右转函数 - 添加障碍物检测
void motorsRight() {
  if (obstacleDetected) {
    Serial.println("检测到障碍物，拒绝右转命令");
    SerialBT.println("Obstacle detected, RIGHT command blocked");
    return;
  }
  targetMotorState = RIGHT;
  Serial.println("右转");
  SerialBT.println("Turn RIGHT");
}

// 按钮防抖读取函数
bool readButtonWithDebounce() {
  int reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      if (currentButtonState == LOW) {
        lastButtonState = reading;
        return true;
      }
    }
  }
  
  lastButtonState = reading;
  return false;
}

// 非阻塞电机测试相关变量
unsigned long testStepStartTime = 0;
int testStep = 0;
bool testStepInProgress = false;

// 修改电机测试函数为非阻塞版本
void runMotorTest() {
  if (!testRunning) {
    // 开始测试
    Serial.println("开始电机测试...");
    SerialBT.println("Starting motor test...");
    testRunning = true;
    testStep = 1;
    testStepStartTime = millis();
    testStepInProgress = false;
  }
}

// 添加非阻塞测试更新函数
void updateMotorTest() {
  if (!testRunning) return;
  
  unsigned long currentTime = millis();
  
  switch(testStep) {
    case 1: // 测试1：前进
      if (!testStepInProgress) {
        Serial.println("测试1：前进");
        motorsForward();
        testStepInProgress = true;
        testStepStartTime = currentTime;
      } else if (currentTime - testStepStartTime >= 3000) {
        motorsStop();
        testStepInProgress = false;
        testStepStartTime = currentTime;
        testStep = 2; // 进入停止等待
      }
      break;
      
    case 2: // 停止等待1
      if (currentTime - testStepStartTime >= 2000) {
        testStep = 3;
        testStepInProgress = false;
      }
      break;
      
    case 3: // 测试2：后退
      if (!testStepInProgress) {
        Serial.println("测试2：后退");
        motorsBackward();
        testStepInProgress = true;
        testStepStartTime = currentTime;
      } else if (currentTime - testStepStartTime >= 3000) {
        motorsStop();
        testStepInProgress = false;
        testStepStartTime = currentTime;
        testStep = 4;
      }
      break;
      
    case 4: // 停止等待2
      if (currentTime - testStepStartTime >= 2000) {
        testStep = 5;
        testStepInProgress = false;
      }
      break;
      
    case 5: // 测试3：左转
      if (!testStepInProgress) {
        Serial.println("测试3：左转");
        motorsLeft();
        testStepInProgress = true;
        testStepStartTime = currentTime;
      } else if (currentTime - testStepStartTime >= 3000) {
        motorsStop();
        testStepInProgress = false;
        testStepStartTime = currentTime;
        testStep = 6;
      }
      break;
      
    case 6: // 停止等待3
      if (currentTime - testStepStartTime >= 2000) {
        testStep = 7;
        testStepInProgress = false;
      }
      break;
      
    case 7: // 测试4：右转
      if (!testStepInProgress) {
        Serial.println("测试4：右转");
        motorsRight();
        testStepInProgress = true;
        testStepStartTime = currentTime;
      } else if (currentTime - testStepStartTime >= 3000) {
        motorsStop();
        testStepInProgress = false;
        testStepStartTime = currentTime;
        testStep = 8;
      }
      break;
      
    case 8: // 最终停止等待
      if (currentTime - testStepStartTime >= 2000) {
        Serial.println("电机测试完成！");
        SerialBT.println("Motor test completed!");
        testRunning = false;
        testStep = 0;
      }
      break;
  }
}

// 修改处理蓝牙命令函数 - 保持原有逻辑
void processBluetoothCommand() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.print("收到蓝牙命令: ");
    Serial.print(command);
    Serial.print(" (ASCII: ");
    Serial.print((int)command);
    Serial.println(")");
    
    // 紧急停止：即使在测试中也允许停止
    if (command == 'S' || command == 's') {
      motorsStop();
      if (testRunning) {
        testRunning = false;
        testStep = 0;
        Serial.println("测试被中止");
        SerialBT.println("Test aborted");
      }
      return;
    }
    
    // 如果正在测试中，除停止外忽略其他蓝牙命令
    if (testRunning) {
      SerialBT.println("Test running, only STOP (S) command accepted");
      return;
    }
    
    switch(command) {
      case 'F':  // 前进
      case 'f':
        motorsForward();
        break;
        
      case 'B':  // 后退
      case 'b':
        motorsBackward();
        break;
        
      case 'L':  // 左转
      case 'l':
        motorsLeft();
        break;
        
      case 'R':  // 右转
      case 'r':
        motorsRight();
        break;
        
      // 移除所有速度控制case，包括0-9和:
      // 如果收到数字命令，给出提示
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
      case ':':
        Serial.println("速度控制已禁用，使用固定速度");
        SerialBT.println("Speed control disabled, using fixed speed");
        break;
        
      default:
        Serial.print("未知命令，ASCII码: ");
        Serial.println((int)command);
        SerialBT.print("Unknown command: ");
        SerialBT.println(command);
        SerialBT.println("Commands: F=Forward, B=Backward, L=Left, R=Right, S=Stop");
        break;
    }
  }
}

void loop() {
  // 检查障碍物（最高优先级）
  checkObstacle();
  
  // 处理蓝牙命令
  processBluetoothCommand();
  
  // 更新电机速度（软启动/软停止）
  updateMotorSpeed();
  
  // 更新非阻塞测试
  updateMotorTest();
  
  // 检测按钮按下（硬件测试）
  if (readButtonWithDebounce() && !testRunning) {
    runMotorTest();
  }
  
  delay(10);
}



