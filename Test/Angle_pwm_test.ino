/***************************************************************
Function: adjust velocity of motor with respect to the angle of IMU
balance control as a linear inverted pendulum.
Time: 20190516
****************************************************************/

#include <FlexiTimer2.h>					//Timer library
#include <Wire.h>							//IIC communication library
#include "I2Cdev.h"							//used for MPU6050
#include "MPU6050_6Axis_MotionApps20.h"		//used for MPU6050
#include "KalmanFilter.h"					//used for MPU6050

//Pins define
const int PWM_L = 5;				//connect to motor driver ENA pin
const int IN1_L = 36;				//motor driver(IN1)
const int IN2_L = 37;				//motor driver(IN2)

int Angle_PWM = 0;

//Objects 
MPU6050 IMU;
int16_t ax, ay, az, gx, gy, gz;

KalmanFilter KF;
float K1 = 0.05, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1;
float dt = 0.005;			//fliter sampping time 5ms
int addr = 0;

void setup()
{
	//pins about motor drivers
	pinMode(PWM_L, OUTPUT);
	pinMode(IN1_L, OUTPUT);			//IN1&IN2,01 forward,10 backward
	pinMode(IN2_L, OUTPUT);
	analogWrite(PWM_L, 0);
	digitalWrite(IN1_L, LOW);
	digitalWrite(IN2_L, LOW);

	Wire.begin();
	Serial.begin(9600);
	delay(1000);
	IMU.initialize();
	delay(200);
	FlexiTimer2::set(5,control);
	FlexiTimer2::start();
}

void loop()
{}

void control()
{
	sei();
	IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	KF.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
	Serial.print("Angle: "); Serial.print(KF.angle); Serial.print("  ");

	Angle_PWM = angle_2_pwm(KF.angle);
	set_PWM(Angle_PWM);
	Serial.print("PWM: "); Serial.print(Angle_PWM); Serial.println();
}

/*************************************************************
Function: calculate the PWM from angle of  twist
		angle			PWM
	 -20 ~ +20      -250 ~ 250
**************************************************************/
int angle_2_pwm(float angle)
{
	//limit the amplitude of angle value
	int max_angle = 20;
	if (angle > max_angle)	angle = max_angle;
	if (angle < -max_angle)	angle = -max_angle;

	static int tmp_angle;
	tmp_angle = 12.5*angle;		//12.5 comes from K = Max_PWM/Max_angle = 250/20 = 12.5
	return tmp_angle;
}

/*************************************************************
Function: set PWM value to motor driver
		  0~250 reprented for duty cycle of 0~100%
**************************************************************/
void set_PWM(int motor_PWM_L)
{
	//limit the amplitude of PWM value, or use map()
	int max = 250;
	if (motor_PWM_L > max)	motor_PWM_L = max;
	if (motor_PWM_L < -max)	motor_PWM_L = -max;

	//write PWM value to motor driver board
	if (motor_PWM_L > 0)
	{
		digitalWrite(IN1_L, HIGH);
		digitalWrite(IN2_L, LOW);
	}
	else
	{
		digitalWrite(IN1_L, LOW);
		digitalWrite(IN2_L, HIGH);
	}
	analogWrite(PWM_L, abs(motor_PWM_L));
}
