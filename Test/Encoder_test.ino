/*****************************************************
外部中断读取编码器信号
******************************************************/

const int ENCODER = 18;
const int DIRECTION = 23;
volatile long encoder_cnt = 0;

/***************** ISR外中断读取编码器 **************/
void READ_ENCODER()
{
	if (digitalRead(ENCODER) == HIGH)
	{
		if (digitalRead(DIRECTION) == HIGH)
			encoder_cnt++;
		else
			encoder_cnt--;
	}
	else
	{
		if (digitalRead(DIRECTION) == LOW)
			encoder_cnt++;
		else
			encoder_cnt--;
	}
}

void setup()
{
	pinMode(ENCODER, INPUT);
	pinMode(DIRECTION, INPUT);
	Serial.begin(115200);
	attachInterrupt(5, READ_ENCODER, CHANGE);
}

void loop()
{
	Serial.print("encoder_cnt");
	Serial.println(encoder_cnt/27);
	delay(200);
}
