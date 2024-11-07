// 使用Arduino+Can通讯模块控制电机

#include <SPI.h>
#include "DFRobot_MCP2515.h"

#define CAN_CS_PIN 10  // 定义片选引脚为 D10

DFRobot_MCP2515 can(CAN_CS_PIN);  

void setup() 
{
  Serial.begin(115200);  

  // 使用 begin() 方法配置 CAN 控制器波特率
  while (can.begin(CAN_1000KBPS) != CAN_OK) {  // 修改波特率为 1 Mbps
    Serial.println("CAN 初始化失败！请重新初始化。");
    delay(3000);  // 如果 CAN 初始化失败，等待 3 秒后重试
  }
  Serial.println("CAN 初始化成功！");

  // 初始化掩码和过滤器，用于过滤无关消息
  if (can.initMask(MCP2515_RXM0, 0, 0x000) != MCP2515_OK) {  // 放宽掩码以接收所有消息
    Serial.println("掩码初始化失败！");
  }
  if (can.initFilter(MCP2515_RXF0, 0, 0x001) != MCP2515_OK) {  // 修改过滤器以匹配电机的默认 ID 为 0x001
    Serial.println("过滤器初始化失败！");
  }
}

void loop() 
{
  uint8_t data[] = {0x01, 0x01, 0x20, 0x00, 0x00, 0x00, 0x3F};  // 
  if (can.sendMsgBuf(0x001, 0, sizeof(data), data) == MCP2515_OK) {  // 修改 ID 为 0x001 以匹配电机的默认 ID
    Serial.println("数据发送成功！");
  } else {
    Serial.println("数据发送失败！");
  }

  // 检查是否接收到 CAN 消息
  if (can.checkReceive() == CAN_MSGAVAIL) {
    uint8_t len = 0;
    uint8_t buf[8];

    // 读取消息内容
    if (can.readMsgBuf(&len, buf) == MCP2515_OK) {
      uint32_t id = can.getCanId();
      Serial.print("接收到的数据 ID: ");
      Serial.println(id, HEX);
      Serial.print("数据: ");
      for (int i = 0; i < len; i++) {
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  } else {
    Serial.println("没有接收到 CAN 消息。");
  }

  // 检查 CAN 总线错误
  if (can.checkError() != CAN_OK) {
    Serial.println("CAN 总线存在错误！");
  }

  delay(1000);  // 每隔 1 秒发送一次
}
