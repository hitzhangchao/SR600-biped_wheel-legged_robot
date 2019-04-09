/*
This code is used for connecting arduino to serial mpu6050 module, and test in arduino uno R3 board.
connect map:
arduino   mpu6050 module
VCC    5v/3.3v
TX     RX<-0
TX     TX->1
GND    GND
note: 
because arduino download and mpu6050 are using the same serial port, you need to un-connect 6050 module when you want to download program to arduino.
 Created 14 Nov 2013
 by Zhaowen
 
 serial mpu6050 module can be found in the link below:
 http://item.taobao.com/item.htm?id=19785706431
 */

unsigned char Re_buf[11],counter=0;
unsigned char sign=0;
float a[3],w[3],angle[3],T;
void setup() {
  // initialize serial:
  Serial.begin(115200);
}

void loop() {
  if(sign)
  {  
     sign=0;
     if(Re_buf[0]==0x55)      //检查帧头
     {  
	switch(Re_buf [1])
	{
	case 0x51:
		a[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*16;
		a[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*16;
		a[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*16;
		T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
		break;
	case 0x52:
		w[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*2000;
		w[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*2000;
		w[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*2000;
		T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
		break;
	case 0x53:
        	angle[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*180;
		angle[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*180;
		angle[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*180;
		T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
                Serial.print("a:");
                Serial.print(a[0]);Serial.print(" ");
                Serial.print(a[1]);Serial.print(" ");
                Serial.print(a[2]);Serial.print(" ");
                Serial.print("w:");
                Serial.print(w[0]);Serial.print(" ");
                Serial.print(w[1]);Serial.print(" ");
                Serial.print(w[2]);Serial.print(" ");
                Serial.print("angle:");
                Serial.print(angle[0]);Serial.print(" ");
                Serial.print(angle[1]);Serial.print(" ");
                Serial.print(angle[2]);Serial.print(" ");
                Serial.print("T:");
                Serial.println(T);
                break;
	} 
    }
  } 
}

void serialEvent() {
  while (Serial.available()) {
    
    //char inChar = (char)Serial.read(); Serial.print(inChar); //Output Original Data, use this code 
  
    Re_buf[counter]=(unsigned char)Serial.read();
    if(counter==0&&Re_buf[0]!=0x55) return;      //第0号数据不是帧头              
    counter++;       
    if(counter==11)             //接收到11个数据
    {    
       counter=0;               //重新赋值，准备下一帧数据的接收 
       sign=1;
    }
      
  }
}

