#include <SPI.h>
#include "DFRobot_MCP2515.h"

#define CAN_CS_PIN 10  // 定义片选引脚为 D10

DFRobot_MCP2515 can(CAN_CS_PIN);  // 实例化 MCP2515 对象

void setup() 
{
  Serial.begin(115200);  // 初始化串口，用于查看接收到的数据
  
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
  // 检查是否有接收到的 CAN 消息
  if (can.checkReceive() == CAN_MSGAVAIL) {
    uint8_t len = 0;
    uint8_t buf[8];

    // 读取消息内容
    if (can.readMsgBuf(&len, buf) == MCP2515_OK) {
      // 获取消息的 ID
      uint32_t id = can.getCanId();
      Serial.print("接收到的数据 ID: ");
      Serial.println(id, HEX);
      
      Serial.print("数据: ");
      for (int i = 0; i < len; i++) {
        Serial.print((char)buf[i]);  // 将数据按字符打印
      }
      Serial.println();
    }
  }
}
