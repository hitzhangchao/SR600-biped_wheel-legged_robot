/***************************************************************
IDE: VS for Arduino
Hardware: Arduino Mega 2560

Function: Software for SR600 Biped wheel-legged robot to implement
balance control as a linear inverted pendulum.
Time: 20190408
****************************************************************/

#include <FlexiTimer2.h>					//Timer library
#include <Wire.h>							//IIC communication library
#include "I2Cdev.h"							//used for MPU6050
#include "MPU6050_6Axis_MotionApps20.h"		//used for MPU6050
#include "KalmanFilter.h"					//used for MPU6050

//Pins define
//SDA = 20,SCL = 21,IIC pin(IMU)
const int ENCODER_L = 19;			//Encoder_left pin(external interrupt INT.4)
const int DIRECTION_L = 22;			//direction discrimination
const int ENCODER_R = 18;			//Encoder_right pin(INT.5)
const int DIRECTION_R = 23;			//direction discrimination
const int PWM_L = 9;				//connect to motor driver ENA pin
const int PWM_R = 10;				//connect to motor driver ENB pin
const int IN1_L = 26;				//motor driver(IN1)
const int IN2_L = 27;				//motor driver(IN2)
const int IN1_R = 28;				//motor driver(IN3)
const int IN2_R = 29;				//motor driver(IN4)

//Objects 
MPU6050 IMU;
int16_t ax, ay, az, gx, gy, gz;

KalmanFilter KF;
float K1 = 0.05, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1;
float dt = 0.005;			//fliter sampping time 5ms
int addr = 0;

int Angle_PWM, Velocity_PWM, Turn_PWM;					//output of PD/PI/PD controller
int Motor_PWM_L, Motor_PWM_R;							//final PWM add to motor
volatile long Encoder_cnt_L = 0, Encoder_cnt_R = 0;		//encoder pulse num.
int Velocity_L = 0, Velocity_R = 0;						//velocity calculated from encoder
float Angle;


//PID parameters
float Angle_Kp = 20, Angle_Kd = 0;			//angle PD controller para.
float Velocity_Kp = 0, Velocity_Ki = 0;		//velocity PI controller para.


void setup()
{
	//pins about encoders
	pinMode(ENCODER_L, INPUT);
	pinMode(DIRECTION_L, INPUT);
	pinMode(ENCODER_R, INPUT);
	pinMode(DIRECTION_R, INPUT);

	//pins about motor drivers
	pinMode(PWM_L, OUTPUT);
	pinMode(PWM_R, OUTPUT);
	pinMode(IN1_L, OUTPUT);			//IN1&IN2,01 forward,10 backward
	pinMode(IN2_L, OUTPUT);
	pinMode(IN1_R, OUTPUT);
	pinMode(IN2_R, OUTPUT);
	analogWrite(PWM_L, 0);
	analogWrite(PWM_R, 0);
	digitalWrite(IN1_L, LOW);
	digitalWrite(IN2_L, LOW);
	digitalWrite(IN1_R, LOW);
	digitalWrite(IN2_R, LOW);

	Wire.begin();					//IIC begin
	Serial.begin(9600);				//Serial begin
	delay(1500);
	IMU.initialize();				//IMU object initialize
	delay(20);
	FlexiTimer2::set(5, control);	//Timer, 5ms, ISR:control
	FlexiTimer2::start();
	attachInterrupt(4, read_encoder_left, CHANGE);		//INT4, read left encoder
	attachInterrupt(5, read_encoder_right, CHANGE);		//INT5, read right encoder
} 


void loop()
{

}

/*************************************************************
Function: ISR of MsTimer2
**************************************************************/
void control()
{
	sei();
	static int vel_ctl_cnt;												//to assure 40ms velocity control period
	static int turn_ctl_cnt;											//to assure 40ms turn control period
	IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);						//get data from MPU6050
	KF.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);	//kalman filter used to calculate angle and gyro

	Angle_PWM = angle_PD_control(KF.angle, KF.Gyro_x);					//angle PD control,period 5ms
	//test point
	Serial.print("Angle ");
	Serial.print(KF.angle);
	Serial.print(" ");

	if (++vel_ctl_cnt >= 8)
	{
		Velocity_L = Encoder_cnt_L; 
		Encoder_cnt_L = 0;
		Velocity_R = Encoder_cnt_R; 
		Encoder_cnt_R = 0;
		Velocity_PWM = velocity_PI_control(Velocity_L, Velocity_R);		//velocity PI control, period 30ms;
		vel_ctl_cnt = 0;
	}
	//test point
	Serial.print("Velocity ");
	Serial.println(Velocity_L);

	Motor_PWM_L = Angle_PWM - Velocity_PWM;
	Motor_PWM_R = Angle_PWM - Velocity_PWM;
	set_PWM(Motor_PWM_L, Motor_PWM_R);
}

/*************************************************************
Function: ISR of INT.4,to read encoder pulse num.
**************************************************************/
void read_encoder_left()
{
	if (digitalRead(ENCODER_L) == HIGH)
	{
		if (digitalRead(DIRECTION_L) == HIGH)
			Encoder_cnt_L++;
		else
			Encoder_cnt_L--;
	}
	else
	{
		if (digitalRead(DIRECTION_L) == LOW)
			Encoder_cnt_L++;
		else
			Encoder_cnt_L--;
	}
}

/*************************************************************
Function: ISR of INT.5, to read encoder pulse num.
**************************************************************/
void read_encoder_right()
{
	if (digitalRead(ENCODER_R) == HIGH)
	{
		if (digitalRead(DIRECTION_R) == HIGH)
			Encoder_cnt_R--;
		else
			Encoder_cnt_R++;
	}
	else
	{
		if (digitalRead(DIRECTION_R) == LOW)
			Encoder_cnt_R--;
		else
			Encoder_cnt_R++;
	}
}

/*************************************************************
Function: robot angle balance control through PD controller
Input: angle,angular_velocity(gyro)
Output: PWM
**************************************************************/
int angle_PD_control(float angle, float gyro)
{
	float bias;
	int angle_PWM;
	bias = angle - 0;			//bias of angle,0 is the set value
	angle_PWM = Angle_Kp*bias + Angle_Kd*gyro;
	return angle_PWM;
}

/*************************************************************
Function: robot velocity control through PI controller
Input: encoder(velocity)
Output: PWM
**************************************************************/
int velocity_PI_control(int Velocity_L, int Velocity_R)
{
	static float velocity_bias, velocity_bias_filtered = 0, velocity_bias_integral = 0;
	int velocity_PWM;
	velocity_bias = (Velocity_L + Velocity_R) - 0;						//bias of velocity,0 is target value
	velocity_bias_filtered *= 0.8;
	velocity_bias_filtered += velocity_bias*0.2;						//low-pass filter
	velocity_bias_integral += velocity_bias_filtered;					//velocity bias integral
	//limit the amplitude of velocity bias intgral
	if (velocity_bias_integral > 20000)
		velocity_bias_integral = 20000;
	if (velocity_bias_integral < -20000)
		velocity_bias_integral = -20000;
	velocity_PWM = Velocity_Kp*velocity_bias_filtered + Velocity_Ki*velocity_bias_integral;
	return velocity_PWM;
}

/*************************************************************
Function: set PWM value to motor driver
0~255 reprented for duty cycle of 0~100%
**************************************************************/
void set_PWM(int &Motor_PWM_L, int &Motor_PWM_R)
{
	//limit the amplitude of PWM value, or use map()
	int max = 255;
	if (Motor_PWM_L > max)	Motor_PWM_L = max;
	if (Motor_PWM_L < -max)	Motor_PWM_L = -max;
	if (Motor_PWM_R > max)	Motor_PWM_R = max;
	if (Motor_PWM_R < -max)	Motor_PWM_R = -max;

	if (Motor_PWM_L > 0)
		digitalWrite(IN1_L, HIGH), digitalWrite(IN2_L, LOW);
	else
		digitalWrite(IN1_L, LOW), digitalWrite(IN2_L, HIGH);
	analogWrite(PWM_L, abs(Motor_PWM_L));

	if (Motor_PWM_R > 0)
		digitalWrite(IN1_R, LOW), digitalWrite(IN2_R, HIGH);
	else
		digitalWrite(IN1_R, HIGH), digitalWrite(IN2_R, LOW);
	analogWrite(PWM_R, abs(Motor_PWM_R));
}

/*************************************************************
Function: send PID parameters through serial port
Communication Protocal:
	start  Angle_Kp  Angle_Kd  Velocity_Kp  Velocity_Ki  stop
	0xFF      X          X         X             X       0xFE 
**************************************************************/
void serialEvent()
{
	static unsigned char flag, data;
	static int i = 0, j = 0;
	static unsigned char receive_data[6];
	while (Serial.available())
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
		}
	}
}
