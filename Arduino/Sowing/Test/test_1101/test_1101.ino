#include <AccelStepper.h>

int EN_PIN = 7;    // 使能引脚
int DIR_PIN = 6;   // 方向引脚
int STEP_PIN = 5;  // 脉冲引脚

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() 
{
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);    // 使能步进电机

  stepper.setMaxSpeed(1600); // 设置最大速度为每秒 1600 步
  stepper.setCurrentPosition(0); // 设置初始位置为 0 步
  stepper.setSpeed(-200); // 调整电机转速
}

void loop() 
{
  stepper.runSpeed(); // 持续运行步进电机
}
