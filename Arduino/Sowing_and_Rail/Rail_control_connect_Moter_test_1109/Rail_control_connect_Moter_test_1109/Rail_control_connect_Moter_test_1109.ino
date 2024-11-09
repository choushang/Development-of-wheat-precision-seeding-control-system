// 单片机2：控制导轨电机，接收来自串口的指令并控制电机

#include <AccelStepper.h>

// 步进电机配置
const int xdirPin = 5;     // 方向控制引脚
const int xstepPin = 2;    // 步进控制引脚
const int xenablePin = 8;  // 使能控制引脚
const int buttonPin = 9;   // 接近开关引脚

bool motorAtOrigin = false;
AccelStepper stepper1(1, xstepPin, xdirPin); // 建立步进电机对象

void setup() 
{
    Serial.begin(9600);  // 初始化硬串口，用于与单片机1通信

    // 步进电机初始化
    pinMode(xstepPin, OUTPUT);
    pinMode(xdirPin, OUTPUT);
    pinMode(xenablePin, OUTPUT);
    digitalWrite(xenablePin, LOW); // 启用电机驱动板

    pinMode(buttonPin, INPUT_PULLUP);  // 启用内部上拉电阻，用于按压开关

    stepper1.setMaxSpeed(2500.0);     
    stepper1.setAcceleration(800.0);  

    // 检查步进电机是否到达原点
    while (digitalRead(buttonPin) == HIGH) {
        stepper1.setSpeed(2000);  // 以较低速度向原点移动
        stepper1.runSpeed();
    }
    motorAtOrigin = true;
    stepper1.setCurrentPosition(0); // 设置当前位置为原点
    stepper1.setSpeed(0); // 停止电机
    Serial.println("电机已到达原点。");
    stepper1.disableOutputs();
}

void loop() 
{
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        parseAndExecuteCommand(input);
    }

    // 步进电机非阻塞运行
    if (motorAtOrigin && stepper1.distanceToGo() != 0) {
        stepper1.run();
    }
}

// 解析串口输入并执行相应的命令
void parseAndExecuteCommand(String input)
{
    if (input.startsWith("D")) {
        String distanceString = input.substring(1);
        float distanceValue = distanceString.toFloat();
        if (distanceValue > 0) {
            movePlatformToDistance(distanceValue);
        }
    }
}

void movePlatformToDistance(float distance_mm) 
{
    if (motorAtOrigin) {
        long stepsToMove = distance_mm * 160; // 每毫米对应 160 步，8 细分
        stepper1.moveTo(stepper1.currentPosition() - stepsToMove);
    }
}
