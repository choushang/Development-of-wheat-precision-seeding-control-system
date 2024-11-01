// 导轨电机基础测试，限位开关+步进设置

#include <AccelStepper.h>  

// 定义电机控制用常量
const int xdirPin = 5;     // 方向控制引脚
const int xstepPin = 2;    // 步进控制引脚
const int xenablePin = 8;  // 使能控制引脚
const int buttonPin = 9;  // 接近开关引脚

bool motorTriggered = false;  // 标志变量
const int moveSteps = -20000;    //运行步数

AccelStepper stepper1(1,xstepPin,xdirPin);//建立步进电机对象

void setup() {
  pinMode(xstepPin, OUTPUT);     // Arduino控制A4988步进引脚为输出模式
  pinMode(xdirPin, OUTPUT);      // Arduino控制A4988方向引脚为输出模式
  pinMode(xenablePin, OUTPUT);   // Arduino控制A4988使能引脚为输出模式
  digitalWrite(xenablePin, LOW); // 将使能控制引脚设置为低电平从而让电机驱动板进入工作状态

  pinMode(buttonPin, INPUT_PULLUP);  // 启用内部上拉电阻，用于按压开关

  stepper1.setMaxSpeed(2000.0);     // 设置电机最大速度
  stepper1.setAcceleration(800.0);  // 设置电机加速度
  Serial.begin(115200);  // 初始化串口通信
}


void loop() 
{
  int buttonState = digitalRead(buttonPin);  // 读取按压开关的状态

  if (buttonState == HIGH && motorTriggered == false) 
  {
    stepper1.setSpeed(1600);  
    stepper1.runSpeed();  
  }

  if ( buttonState == LOW && motorTriggered == false )
  { 
    motorTriggered = true;         // 设置标志，防止重复触发
    stepper1.setSpeed(2000);        
    stepper1.moveTo(stepper1.currentPosition() + moveSteps); 
  } 

  if (motorTriggered) 
  {
    stepper1.run();  // 使用run()移动到目标位置

    // 当电机到达目标位置后，停止电机
    if (stepper1.distanceToGo() == 0) 
    {
      stepper1.disableOutputs();   // 禁用电机输出，停止转动
    }
  }
}