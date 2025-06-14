#include <Arduino.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>

// 电机1引脚定义 (右电机 - Right Motor)
#define MOTOR_RIGHT_IN1 32    // 右电机的IN1引脚
#define MOTOR_RIGHT_IN2 33    // 右电机的IN2引脚

// 电机2引脚定义 (左电机 - Left Motor)  
#define MOTOR_LEFT_IN1 25     // 左电机的IN1引脚
#define MOTOR_LEFT_IN2 26     // 左电机的IN2引脚

// 超声波模块引脚定义
#define TRIG_PIN 27           // 超声波模块的Trig引脚
#define ECHO_PIN 14           // 超声波模块的Echo引脚

// 舵机引脚定义
#define SERVO_PIN 13          // MG996R舵机信号线连接到GPIO13

// PWM参数设置
#define PWM_FREQ 1000         // PWM频率
#define PWM_RESOLUTION 8      // PWM分辨率，8位，0-255

// PWM通道分配 - 避免与舵机库冲突
#define PWM_CHANNEL_RIGHT_1 4  // 右电机-IN1的PWM通道（避开0-3通道）
#define PWM_CHANNEL_RIGHT_2 5  // 右电机-IN2的PWM通道
#define PWM_CHANNEL_LEFT_1 6   // 左电机-IN1的PWM通道
#define PWM_CHANNEL_LEFT_2 7   // 左电机-IN2的PWM通道

// 超声波参数
#define MAX_DISTANCE 400      // 最大测距范围(cm)
#define TIMEOUT_US 30000      // 超时时间(微秒)
#define OBSTACLE_THRESHOLD 5  // 障碍物检测阈值(cm)

// 舵机参数
#define SERVO_CENTER 90       // 舵机中心位置
#define SERVO_TURN_ANGLE 10   // 转向时舵机偏转角度

// 创建蓝牙串口对象
BluetoothSerial SerialBT;

// 创建舵机对象
Servo servo;

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

// 动作阶段枚举
enum ActionPhase {
  PHASE_IDLE,           // 空闲状态
  PHASE_SERVO_MOVING,   // 舵机运动阶段
  PHASE_MOTOR_RUNNING,  // 电机运行阶段
  PHASE_MOTOR_STOPPING, // 电机停止阶段
  PHASE_SERVO_RETURNING // 舵机回正阶段
};

MotorState currentMotorState = STOPPED;
MotorState targetMotorState = STOPPED;

// 舵机控制变量
int currentServoAngle = SERVO_CENTER;    // 当前舵机角度
int targetServoAngle = SERVO_CENTER;     // 目标舵机角度
bool servoNeedsUpdate = false;           // 舵机是否需要更新

// 动作阶段控制变量
ActionPhase currentPhase = PHASE_IDLE;
MotorState pendingMotorState = STOPPED;  // 等待执行的电机状态
unsigned long phaseStartTime = 0;        // 阶段开始时间
unsigned long servoMoveDelay = 500;      // 舵机运动等待时间(ms)
unsigned long motorStopDelay = 300;      // 电机停止等待时间(ms)

// 函数声明
void motorsStop();
void motorsForward();
void motorsBackward();
void motorsLeft();
void motorsRight();
void processBluetoothCommand();
void updateMotorSpeed();
void setMotorDirection(MotorState direction);
float measureDistance();
void checkObstacle();
void updateServoPosition();
void updateActionPhase();
void startTurnSequence(MotorState turnDirection);
void startStopSequence();

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
  Serial.println(" (70% 功率)");  Serial.println("障碍物检测已启用，阈值: 5cm");
    // 配置超声波引脚
  pinMode(TRIG_PIN, OUTPUT);  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  Serial.println("开始配置PWM通道和电机引脚...");
  
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
  
  // 立即将所有PWM输出设置为0，确保电机不会意外启动
  ledcWrite(PWM_CHANNEL_RIGHT_1, 0);
  ledcWrite(PWM_CHANNEL_RIGHT_2, 0);
  ledcWrite(PWM_CHANNEL_LEFT_1, 0);
  ledcWrite(PWM_CHANNEL_LEFT_2, 0);
  
  Serial.println("所有电机PWM通道已初始化为0");
  delay(100);  // 短暂延时确保设置生效
  
  // 初始化舵机（在电机PWM设置之后）
  Serial.println("正在初始化舵机...");
  servo.attach(SERVO_PIN);
  delay(100);  // 等待舵机库初始化
  servo.write(SERVO_CENTER);  // 舵机初始化到中心位置
  Serial.println("舵机已初始化到中心位置(90度)");
  delay(500);  // 等待舵机到位
  
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
      }    } else if (distance > OBSTACLE_THRESHOLD || distance == -1) {
      if (obstacleDetected) {
        obstacleDetected = false;
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
      // 左转：两个电机都前进，但左电机会比右电机慢
      ledcWrite(PWM_CHANNEL_RIGHT_2, 0);
      ledcWrite(PWM_CHANNEL_LEFT_2, 0);
      break;
      
    case RIGHT:
      // 右转：两个电机都前进，但右电机会比左电机慢
      ledcWrite(PWM_CHANNEL_RIGHT_2, 0);
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
    return;
  }
  
  lastSpeedUpdate = millis();
  
  // 只在空闲状态或电机运行阶段才更新电机
  if (currentPhase != PHASE_IDLE && currentPhase != PHASE_MOTOR_RUNNING && currentPhase != PHASE_MOTOR_STOPPING) {
    return;
  }
  
  if (targetMotorState == STOPPED) {
    if (currentSpeed > 0) {
      currentSpeed -= speedStep * 2;
      if (currentSpeed < 0) currentSpeed = 0;
      
      if (currentSpeed == 0) {
        currentMotorState = STOPPED;
        setMotorDirection(STOPPED);
      }
    }
  } else {
    if (currentMotorState != targetMotorState) {
      currentSpeed = minSpeed;
      currentMotorState = targetMotorState;
      setMotorDirection(currentMotorState);
    } else if (currentSpeed < maxMotorSpeed) {
      currentSpeed += speedStep;
      if (currentSpeed > maxMotorSpeed) {
        currentSpeed = maxMotorSpeed;
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
        // 左转：右电机全速，左电机减速
        ledcWrite(PWM_CHANNEL_RIGHT_1, currentSpeed);
        ledcWrite(PWM_CHANNEL_LEFT_1, currentSpeed * 0.6);  // 左电机60%速度
        break;
        
      case RIGHT:
        // 右转：左电机全速，右电机减速
        ledcWrite(PWM_CHANNEL_RIGHT_1, currentSpeed * 0.6);  // 右电机60%速度
        ledcWrite(PWM_CHANNEL_LEFT_1, currentSpeed);
        break;
    }
  }
}

// 更新舵机位置
void updateServoPosition() {
  if (servoNeedsUpdate) {
    Serial.print("舵机开始转动：从 ");
    Serial.print(currentServoAngle);
    Serial.print("度 到 ");
    Serial.print(targetServoAngle);
    Serial.println("度");
    
    currentServoAngle = targetServoAngle;
    servo.write(currentServoAngle);
    servoNeedsUpdate = false;
    
    Serial.print("舵机转动完成，当前角度: ");
    Serial.print(currentServoAngle);
    Serial.println("度");
    
    // 检查舵机是否正常响应
    if (currentServoAngle == SERVO_CENTER) {
      Serial.println("舵机回中完成");
    } else {
      Serial.println("舵机转向完成");
    }
  }
}

// 开始转向序列：舵机先动，然后电机
void startTurnSequence(MotorState turnDirection) {
  if (currentPhase != PHASE_IDLE) {
    Serial.println("动作进行中，忽略新命令");
    return;
  }
  
  pendingMotorState = turnDirection;
  currentPhase = PHASE_SERVO_MOVING;
  phaseStartTime = millis();
  
  // 设置舵机目标角度
  if (turnDirection == LEFT) {
    targetServoAngle = SERVO_CENTER + SERVO_TURN_ANGLE;
    Serial.println("开始左转序列：舵机先转动");
    SerialBT.println("Starting LEFT turn: Servo moving first");
  } else if (turnDirection == RIGHT) {
    targetServoAngle = SERVO_CENTER - SERVO_TURN_ANGLE;
    Serial.println("开始右转序列：舵机先转动");
    SerialBT.println("Starting RIGHT turn: Servo moving first");
  }
  servoNeedsUpdate = true;
}

// 开始停止序列：电机先停，然后舵机回正
void startStopSequence() {
  if (currentPhase == PHASE_MOTOR_RUNNING) {
    currentPhase = PHASE_MOTOR_STOPPING;
    phaseStartTime = millis();
    targetMotorState = STOPPED;
    Serial.println("开始停止序列：电机先停止");
    SerialBT.println("Starting stop sequence: Motors stopping first");
  }
}

// 更新动作阶段
void updateActionPhase() {
  unsigned long currentTime = millis();
  
  switch (currentPhase) {
    case PHASE_SERVO_MOVING:
      // 等待舵机转动完成
      if (currentTime - phaseStartTime >= servoMoveDelay) {
        currentPhase = PHASE_MOTOR_RUNNING;
        targetMotorState = pendingMotorState;
        Serial.println("舵机转动完成，启动电机");
        SerialBT.println("Servo positioned, starting motors");
      }
      break;
      
    case PHASE_MOTOR_STOPPING:
      // 等待电机停止完成
      if (currentMotorState == STOPPED && currentTime - phaseStartTime >= motorStopDelay) {
        currentPhase = PHASE_SERVO_RETURNING;
        phaseStartTime = currentTime;
        targetServoAngle = SERVO_CENTER;
        servoNeedsUpdate = true;
        Serial.println("电机停止完成，舵机回正");
        SerialBT.println("Motors stopped, servo returning to center");
      }
      break;
      
    case PHASE_SERVO_RETURNING:
      // 等待舵机回正完成
      if (currentTime - phaseStartTime >= servoMoveDelay) {
        currentPhase = PHASE_IDLE;
        Serial.println("停止序列完成");
        SerialBT.println("Stop sequence completed");
      }
      break;
      
    case PHASE_MOTOR_RUNNING:
    case PHASE_IDLE:
    default:
      // 这些状态不需要自动转换
      break;
  }
}

// 双电机停止函数
void motorsStop() {
  if (currentPhase == PHASE_MOTOR_RUNNING && (currentMotorState == LEFT || currentMotorState == RIGHT)) {
    // 如果当前是转向状态，启动停止序列
    startStopSequence();
  } else {
    // 直接停止
    targetMotorState = STOPPED;
    currentPhase = PHASE_IDLE;
    Serial.println("正在停止电机...");
    SerialBT.println("Stopping motors...");
  }
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
  startTurnSequence(LEFT);
}

// 右转函数 - 添加障碍物检测
void motorsRight() {
  if (obstacleDetected) {
    Serial.println("检测到障碍物，拒绝右转命令");
    SerialBT.println("Obstacle detected, RIGHT command blocked");
    return;
  }
  startTurnSequence(RIGHT);
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
      // 紧急停止
    if (command == 'S' || command == 's') {
      motorsStop();
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
  
  // 更新动作阶段控制
  updateActionPhase();
  
  // 更新电机速度（软启动/软停止）
  updateMotorSpeed();
  
  // 更新舵机位置
  updateServoPosition();
  
  delay(10);
}



