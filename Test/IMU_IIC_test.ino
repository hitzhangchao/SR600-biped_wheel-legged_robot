/*
This code is used for connecting arduino and MPU6050 module through IIC.
connect map:
arduino   mpu6050 module
5v/3.3v		VCC
SDA(20)		SDA
SCL(21)		SCL
GND			GND
 */

#include "I2Cdev.h"	
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include "KalmanFilter.h"


MPU6050 imu;
int ax, ay, az, gx, gy, gz;	//acc. and  angular_velocity of the IMU

KalmanFilter KF;
/*Parameters of Kalman Filter*/
float K1 = 0.05;	//weight of acc.
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5, C_0 = 1;
float dt = 0.005;	//'dt'is the sampling time of filter. 5ms
int addr = 0;

void setup() 
{
	Serial.begin(9600);
	Wire.begin();
	delay(100);
	imu.initialize();
	delay(20);
}

void loop() 
{
	imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	KF.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1); //get data through KF
	Serial.print(KF.angle);
	Serial.print(" ");
	Serial.print(KF.accelz);
	Serial.print(" ");
	Serial.print(KF.angle6);
	Serial.print(" ");
	Serial.print(KF.Gyro_x);
	Serial.print(" ");
	Serial.print(KF.Gyro_y);
	Serial.print(" ");
	Serial.print(KF.Gyro_z);
	Serial.println(" ");
}
