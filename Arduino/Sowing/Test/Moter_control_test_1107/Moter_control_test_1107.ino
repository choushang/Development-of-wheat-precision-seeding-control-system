#include <SPI.h>
#include "DFRobot_MCP2515.h"

#define CAN_CS_PIN 10  // 定义片选引脚为 D10

DFRobot_MCP2515 can(CAN_CS_PIN);  // 实例化 MCP2515 对象

void setup() 
{
  Serial.begin(115200);  // 初始化串口，用于查看调试信息

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
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    float rpm = input.toFloat();
    if (rpm > 0) {
      int speedValue = round((rpm * 10) / 9.55);  // 将 rpm 转换为弧度/秒并放大 10 倍，使用四舍五入确保精度

      // 将速度值拆分为高低字节
      uint8_t speedHigh = (speedValue >> 8) & 0xFF;
      uint8_t speedLow = speedValue & 0xFF;

      // 打印用于调试
      Serial.print("输入的 RPM: ");
      Serial.println(rpm);
      Serial.print("转换后的速度值（高位）: ");
      Serial.println(speedHigh, HEX);
      Serial.print("转换后的速度值（低位）: ");
      Serial.println(speedLow, HEX);

      // 发送控制命令到电机
      uint8_t data[] = {0x01, 0x01, 0x08, 0x00, 0x00, speedHigh, speedLow};  // 控制模式：速度控制，方向：顺时针，细分值：8，速度：动态输入
      if (can.sendMsgBuf(0x001, 0, sizeof(data), data) == MCP2515_OK) {  // 修改 ID 为 0x001 以匹配电机的默认 ID
        Serial.println("数据发送成功！");
      } else {
        Serial.println("数据发送失败！");
      }
    } else {
      Serial.println("请输入有效的 RPM 值！");
    }
  }
  delay(1000);  // 每隔 1 秒检查一次
}
