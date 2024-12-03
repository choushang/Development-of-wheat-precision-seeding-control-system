const int Trig_1 = 3;  // 设定SR04连接的Arduino引脚
const int Echo_1 = 4; 
const int Trig_2 = 5;  
const int Echo_2 = 6;  

double distance_1, time_1; 
double distance_2, time_2;

void setup() 
{   
  Serial.begin(9600);  // 初始化串口通信及连接SR04的引脚
  pinMode(Trig_1, OUTPUT); 
  pinMode(Echo_1, INPUT); 
  pinMode(Trig_2, OUTPUT); 
  pinMode(Echo_2, INPUT); 
  delay(!000);
} 

void loop() 
{ 
  if (Serial.available() > 0) 
  {
    String command = Serial.readStringUntil('\n'); // 读取指令
    command.trim(); // 去除多余的空格和换行符
    
    if (command == "CALIBRATE") 
    {
      // 执行标定，读取第一个传感器的距离作为标定值
      double calibrationDistance = measureDistance(Trig_1, Echo_1);
      if (calibrationDistance >= 0) 
      {
        Serial.println("0,0," + String(calibrationDistance)); // 返回标定值
      } 
    }
  }

  // 获取第一个传感器的距离
  distance_1 = measureDistance(Trig_1, Echo_1);
  if (distance_1 >= 0) 
  {
      Serial.print("0,1,");
      Serial.println(distance_1); // 发送第一个传感器的数据
  }

  // 获取第二个传感器的距离（如果需要）
  distance_2 = measureDistance(Trig_2, Echo_2);
  if (distance_2 >= 0) 
  {
      Serial.print("0,2,");
      Serial.println(distance_2); // 发送第二个传感器的数据
  }

  delay(500); 
}

double measureDistance(int trigPin, int echoPin) 
{
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2); 
    digitalWrite(trigPin, HIGH); 
    delayMicroseconds(10); 
    digitalWrite(trigPin, LOW); 
    
    double time = pulseIn(echoPin, HIGH, 30000); // 超时时间30ms
    if (time == 0) 
    {
        return -1; // 超时返回-1
    }
    double distance = time / 58.0; // 将时间转换为距离（单位：厘米）
    return distance;
}
