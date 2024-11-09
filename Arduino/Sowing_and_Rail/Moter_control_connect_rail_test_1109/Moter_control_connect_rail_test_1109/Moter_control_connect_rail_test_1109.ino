// 单片机1：CAN 通信控制器，负责与上位机通信并通过软串口发送控制信号给单片机2

#include <SPI.h>
#include "DFRobot_MCP2515.h"
#include <SoftwareSerial.h>

// CAN 配置
#define CAN_CS_PIN 10
DFRobot_MCP2515 can(CAN_CS_PIN);

// 定义SoftwareSerial，用于与单片机2通信
#define RX_PIN 7
#define TX_PIN 8
SoftwareSerial serial2(RX_PIN, TX_PIN);  // 软件串口，用于与单片机2通信

void setup() 
{
    Serial.begin(115200);  // 初始化串口，用于查看调试信息
    serial2.begin(9600);   // 初始化软串口，用于与单片机2通信

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
    if (Serial.available() > 0) 
    {
        String input = Serial.readStringUntil('\n');
        parseAndSendSerialCommand(input);
    }
}

// 解析串口输入并发送串口命令给单片机2
void parseAndSendSerialCommand(String input)
{
    int rIndex = input.indexOf('R');
    int dIndex = input.indexOf('D');

    if (rIndex != -1 && dIndex != -1) {
        // 解析 RPM 值
        String rpmString = input.substring(rIndex + 1, dIndex);
        float rpmValue = rpmString.toFloat();
        uint8_t speedHigh = 0;
        uint8_t speedLow = 0;

        if (rpmValue > 0) {
            int speedValue = round((rpmValue * 10) / 9.55);
            speedHigh = (speedValue >> 8) & 0xFF;
            speedLow = speedValue & 0xFF;

            // 发送控制命令到电机
            uint8_t data[] = {0x01, 0x00, 0x20, 0x00, 0x00, speedHigh, speedLow};  // 控制模式：速度控制，方向：顺时针，细分值：8，速度：动态输入
            if (can.sendMsgBuf(0x001, 0, sizeof(data), data) == MCP2515_OK) {  // 修改 ID 为 0x001 以匹配电机的默认 ID
                Serial.println("电机 RPM 数据发送成功！");
            }
        } else if (rpmValue == 0) {
            // 发送停止命令到电机
            uint8_t data[] = {0x01, 0x00, 0x08, 0x20, 0x00, 0x00, 0x00};  // 控制模式：速度控制，方向：顺时针，细分值：8，速度：0
            if (can.sendMsgBuf(0x001, 0, sizeof(data), data) == MCP2515_OK) {
                Serial.println("电机停止成功！");
            }
        }

        // 解析距离值并通过串口发送给单片机2
        String distanceString = input.substring(dIndex + 1);
        float distanceValue = distanceString.toFloat();
        if (distanceValue > 0) {
            serial2.print('D');
            serial2.println(distanceValue);
            Serial.println("距离数据发送给单片机2成功！");
        }
    }
}
