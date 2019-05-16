/***************************************************************
* 通过串口发送0~250的PWM值，测试单个直流电机PWM调速
* 配置描述：
			Motor1   ENA=5   N1=36    N2=37
			Motor2   ENA=6   N1=38    N2=39

			IN1    IN2     ENA      电机状态
			0      0       X       刹车
			1      1       X       悬空
			1      0      PWM      正转调速
			0      1      PWM      反转调速
			1      0       1       正转全速
			0      1       1       反转全速
***************************************************************/
const int ENA = 5;
const int IN1 = 36;
const int IN2 = 37;

long PWM_Data = 0;
String Input_string = "";
bool Read_completed = false;
char Finish_flag = '\n';

void setup() {
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(ENA, OUTPUT);
	Serial.begin(9600);
}

void loop()
{
	digitalWrite(IN1, 1);
	digitalWrite(IN2, 0);
	analogWrite(ENA, PWM_Data);
	if (Read_completed == true)
	{
		PWM_Data = string_2_int(Input_string);
		Serial.print(Input_string); Serial.print("; "); Serial.print(PWM_Data); Serial.println();
		Read_completed = false;
		Input_string = "";
	}
}

//serialEvent() is called inside the loop(), so it's not a realtime interrupt
void serialEvent()
{
	while (Serial.available())
	{
		//get the new byte
		char inChar = (char)Serial.read();
		if (inChar != Finish_flag)
		{
			Input_string += inChar;
		}
		// if the incoming character is a newline, set a flag so the main loop can know about it.
		if (inChar == Finish_flag)
		{
			Read_completed = true;
		}
	}
}

//Transform string to int(there still a little problem,when num. value is above 99)
long string_2_int(String &string)
{
	long num = 0;
	for (int i = 0; i < string.length(); i++)
	{
		int tmp = string.charAt(i) - '0';
		num += tmp*pow(10, string.length() - i - 1);
	}
	return num;
}
