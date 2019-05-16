/**********************************************
* Send data via serial port to change control param.
*Communication protocol: num1 num2 ... \n(finish flag)
* Time: 20190516
***********************************************/

const int LED = 13;
long Data = 0;

String Input_string = "";
bool Read_completed = false;
char Finish_flag = '\n';

void setup()
{
	pinMode(LED,OUTPUT);
	Serial.begin(9600);
}

void loop()
{
	digitalWrite(LED,HIGH);
	delay(Data);
	digitalWrite(LED, LOW);
	delay(Data);
	if (Read_completed == true)
	{
		Data = string_2_int(Input_string);
		Serial.print(Input_string); Serial.print("; "); Serial.print(Data); Serial.println();
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
		if(inChar == Finish_flag)
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
		num +=  tmp*pow(10,string.length()-i-1);
	}
	return num;
}
