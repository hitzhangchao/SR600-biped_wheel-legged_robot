/*****************************************************
在Arduino Mega 2560上使用FlexiTimer2定时器库实现定时器中断
******************************************************/

#include <FlexiTimer2.h>

void control()
{
	Serial.begin(9600);
	Serial.println("test");
}

void setup()
{
	Serial.begin(9600);
	FlexiTimer2::set(50, control);
	FlexiTimer2::start();

}

void loop()
{}
