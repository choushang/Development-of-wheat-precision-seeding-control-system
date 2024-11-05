#include <SPI.h>
#include "DFRobot_MCP2515.h"

#define CAN_CS_PIN 10  // 定义片选引脚为 D10

DFRobot_MCP2515 can(CAN_CS_PIN);  // 实例化 MCP2515 对象

void setup() 
{
  Serial.begin(115200);  // 初始化串口，用于查看调试信息
  // 使用 begin() 方法配置 CAN 控制器波特率
  if (can.begin(CAN_500KBPS) != CAN_OK) {
    Serial.println("CAN 初始化失败！");
    while (1);  // 如果 CAN 初始化失败，停止程序
  } else {
    Serial.println("CAN 初始化成功！");
  }
}

void loop() 
{
  // 发送字符串 "OK" 通过 CAN 总线
  uint8_t data[] = {'O', 'K'};  // 要发送的数据
  if (can.sendMsgBuf(0x100, 0, sizeof(data), data) == MCP2515_OK) {
    Serial.println("数据发送成功！");
  } else {
    Serial.println("数据发送失败！");
  }
  delay(1000);  // 每隔 1 秒发送一次
}