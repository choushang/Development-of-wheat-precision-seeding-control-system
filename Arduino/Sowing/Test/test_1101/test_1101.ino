#include <AccelStepper.h>

int EN_PIN = 7;    // 使能引脚
int DIR_PIN = 6;   // 方向引脚
int STEP_PIN = 5;  // 脉冲引脚

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() 
{
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);    // 使能步进电机

  stepper.setMaxSpeed(1200);     // 设置最大速度为每秒 1200 步
}

void loop() 
{
  // 设置速度为正方向 1200 步每秒，持续运行直到达到目标
  stepper.setSpeed(800);
  while (true) {
    stepper.runSpeed();
  }
}
