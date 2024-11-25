// 单片机1：CAN 通信控制器，负责与上位机通信并通过软串口发送控制信号给单片机2

#include <SPI.h>
#include "DFRobot_MCP2515.h"
#include <SoftwareSerial.h>

// CAN 配置
#define CAN_CS_PIN 10
#define MICRO_STEP 32  // 定义细分值
DFRobot_MCP2515 can(CAN_CS_PIN);
bool firstMessageSent = false;

// 定义SoftwareSerial，用于与单片机2通信
#define RX_PIN 7
#define TX_PIN 8
SoftwareSerial serial2(RX_PIN, TX_PIN);  // 软件串口，用于与单片机2通信

String inputString = "";  // 用于接收串口输入的数据
float lastRpmValue = -1; // 上次接收的 RPM 值，初始化为无效值 

void setup() 
{
  Serial.begin(115200);  // 初始化串口，用于查看调试信息
  serial2.begin(9600);   // 初始化软串口，用于与单片机2通信

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
  static bool dataSent = false;
  unsigned long canId = 0xC1;
  uint8_t data_start[8] = {0x00, 0x20, 0x23, 0x00, 0x00, 0x00, 0x00, 0x03}; 
  if (Serial.available() > 0) 
  {
    can.sendMsgBuf(canId, 0, sizeof(data_start), data_start);
    String input = Serial.readStringUntil('\n');   
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

        if (rpmValue > 0) 
        {
            // 计算周期值
            uint16_t period = 300000 / (rpmValue * MICRO_STEP);  

            // 如果上次的 RPM 值有效，且本次 RPM 值与上次的差小于等于 5
            if (lastRpmValue >= 0 && abs(rpmValue - lastRpmValue) <= 5 ) 
            {
                // 构造并发送 RPM + 15 的 CAN 消息
                float adjustedRpmValue = rpmValue + 25;
                uint16_t adjustedPeriod = 300000 / (adjustedRpmValue * MICRO_STEP);
                uint8_t data_adjusted[8];
                constructCANMessage(data_adjusted, adjustedPeriod);
                can.sendMsgBuf(canId, 0, sizeof(data_adjusted), data_adjusted);
            }

            // 构造并发送 CAN 消息
            uint8_t data[8];
            constructCANMessage(data, period);
            can.sendMsgBuf(canId, 0, sizeof(data), data);

            for (int i = 0; i < sizeof(data); i++) 
            {
              Serial.print("0x");
              Serial.print(data[i], HEX);
              Serial.print(" ");
            }
            Serial.println();
        } 
        else if (rpmValue == 0) 
        {
            // 发送停止命令到电机
            uint8_t data[] = {0x00, 0x20, 0x25, 0x20, 0x00, 0x00, 0x00, 0x02};  // 停止命令
            can.sendMsgBuf(canId, 0, sizeof(data), data);
        }

        // 更新上次的 RPM 值
        lastRpmValue = rpmValue;

        // 解析距离值并通过串口发送给单片机2
        String distanceString = input.substring(dIndex + 1);
        float distanceValue = distanceString.toFloat();
        serial2.print('D');
        serial2.println(distanceValue);
    }
}


void constructCANMessage(uint8_t *data, uint16_t period)
{
    data[0] = 0x00;
    data[1] = 0x20;
    data[2] = 0x26;
    data[3] = period & 0xFF;
    data[4] = (period >> 8) & 0xFF;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x02;
}