// 单片机1：CAN 通信控制器，负责与上位机通信并通过软串口发送控制信号给单片机2

#include <SPI.h>
#include "DFRobot_MCP2515.h"
#include <SoftwareSerial.h>

// CAN 配置
#define CAN_CS_PIN 10
DFRobot_MCP2515 can(CAN_CS_PIN);
bool firstMessageSent = false;

// 定义SoftwareSerial，用于与单片机2通信
#define RX_PIN 7
#define TX_PIN 8
SoftwareSerial serial2(RX_PIN, TX_PIN);  // 软件串口，用于与单片机2通信

String inputString = "";  // 用于接收串口输入的数据
uint32_t floatToIEEE754(float value);

void setup() 
{
  Serial.begin(115200);  // 初始化串口，用于查看调试信息
  serial2.begin(9600);   // 初始化软串口，用于与单片机2通信

  // 使用 begin() 方法配置 CAN 控制器波特率
  while (can.begin(CAN_125KBPS) != CAN_OK) 
  {  // 修改波特率为 125 Kbps
      Serial.println("CAN 初始化失败！请重新初始化。");
      delay(3000);  // 如果 CAN 初始化失败，等待 3 秒后重试
  }
    Serial.println("CAN 初始化成功！");
  if (can.initMask(MCP2515_RXM0, 0, 0x000) != MCP2515_OK) 
  {  // 掩码为 0x000，接收所有 ID
    Serial.println("掩码初始化失败！");
  }
  if (can.initFilter(MCP2515_RXF0, 0, 0x000) != MCP2515_OK) 
  {  // 过滤器为 0x000
    Serial.println("过滤器初始化失败！");
  }
}

void loop() 
{
  unsigned long canId = 0xC1;
  uint8_t data[8] = {0x00, 0x20, 0x23, 0x00, 0x00, 0x00, 0x00, 0x03}; 
  if (Serial.available() > 0) 
  {
    can.sendMsgBuf(canId, 0, sizeof(data), data);
    String input = Serial.readStringUntil('\n');   // 电机正转启动
    parseAndSendSerialCommand(input);
  }

}

void parseAndSendSerialCommand(String input)
{
    unsigned long canId = 0xC1;
    int rIndex = input.indexOf('R');
    int dIndex = input.indexOf('D');

    if (rIndex != -1 && dIndex != -1) {
        // 解析 RPM 值
        String rpmString = input.substring(rIndex + 1, dIndex);
        float rpmValue = rpmString.toFloat();

        if (rpmValue > 0) {
            uint32_t ieee754Value = floatToIEEE754(rpmValue);

            // 构造CAN消息
            uint8_t data[8] = {0x00, 0x20, 0x26};  // 固定的头部
            uint8_t ieeeBytes[4];
            // 将 IEEE 754 浮动数值转换为小端格式
            ieeeBytes[0] = (ieee754Value & 0xFF);         // 低位字节
            ieeeBytes[1] = (ieee754Value >> 8) & 0xFF;    // 次低位字节
            ieeeBytes[2] = (ieee754Value >> 16) & 0xFF;   // 次高位字节
            ieeeBytes[3] = (ieee754Value >> 24) & 0xFF;   // 高位字节

            // 按小端格式将字节放入 CAN 消息中
            data[3] = ieeeBytes[0];  // 小端格式：低位字节（00）
            data[4] = ieeeBytes[1];  // 小端格式：次低位字节（00）
            data[5] = ieeeBytes[2];  // 小端格式：次高位字节（34）
            data[6] = ieeeBytes[3];  // 小端格式：高位字节（42）
            data[7] = 0x00;

            // 发送 CAN 消息
            can.sendMsgBuf(canId, 0, sizeof(data), data);
        } else if (rpmValue == 0) {
            // 发送停止命令到电机
            uint8_t data[] = {0x00, 0x20, 0x25, 0x20, 0x00, 0x00, 0x00, 0x02};  // 停止命令
            can.sendMsgBuf(canId, 0, sizeof(data), data);
        }

        // 解析距离值并通过串口发送给单片机2
        String distanceString = input.substring(dIndex + 1);
        float distanceValue = distanceString.toFloat();
        serial2.print('D');
        serial2.println(distanceValue);
    }
}




uint32_t floatToIEEE754(float value) {
    union {
        float f;
        uint32_t i;
    } u;
    u.f = value;
    return u.i;  // 返回 IEEE 754 浮点数的十六进制表示
}
