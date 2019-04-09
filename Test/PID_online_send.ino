/**********************************************
测试通过串口发送PID参数，实现通过串口在线调参
串口通信协议：
        起始位  Kp1  Kd1  Kp2  Ki2  停止位
         0xFF   X    X    X    X    0xFE
***********************************************/
float Angle_Kp = 0, Angle_Kd = 0;
float Velocity_Kp = 0, Velocity_Ki = 0;

void setup()
{
	Serial.begin(9600);
}

void loop()
{
}

void serialEvent()
{
	static unsigned char flag, data;
	static int i = 0, j = 0;
	static unsigned char receive_data[6];
	while (Serial.available() > 0)
	{
		data = Serial.read();
		if (data == 0xFF) flag = 1;			//起始标志位0xFF
		if (data == 0xFE) flag = 2;			//停止标志位0xFE
		if (flag == 1)
		{
			receive_data[i] = data;
			i++;
		}
		else if (flag == 2)
		{
			Angle_Kp = (float)receive_data[1];
			Angle_Kd = (float)receive_data[2] / 100;
			Velocity_Kp = (float)receive_data[3];
			Velocity_Ki = (float)receive_data[4] / 100;
			Serial.print(Angle_Kp);
			Serial.print(" ");
			Serial.print(Angle_Kd);
			Serial.print(" ");
			Serial.print(Velocity_Kp);
			Serial.print(" ");
			Serial.println(Velocity_Ki);
		}
	}
}

