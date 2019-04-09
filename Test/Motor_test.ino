/*程序名称：测试单个直流电机测试程序
  功能描述：电机5s内加速正转到最高速，保持2s，5s内减速到0，停止2s;
            再5s内反转到最高速，保持2s,再5s内减速到0，停止2s.循环。
  配置描述： 
            Motor1  ENA 2    N1=52， N2=53
            Motor2  ENB 3    N3=50， N4=51
            IN1/3   IN2/4   ENA/B  电机状态
              0      0       X       刹车
              1      1       X       悬空
              1      0      PWM      正转调速
              0      1      PWM      反转调速
              1      0       1       正转全速
		          0      1       1       反转全速
***************************************************************/
const int ENA = 2;
const int ENB = 3;
const int IN1 = 52;
const int IN2 = 53;
const int IN3 = 50;
const int IN4 = 51;

void setup() {
  pinMode(IN1,OUTPUT);     
  pinMode(IN2,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENB,OUTPUT);
}

void loop() 
{
  motorAcc(1,5000);
  maxSpeed(1,2000);
  motorDec(1,5000);
  motorRelax(2000);

  motorAcc(0,5000);
  maxSpeed(0,2000);
  motorDec(0,5000);
  motorSwitch(2000);
}

/*motor keep max speed*/
void maxSpeed(int dir,int time)
{
  if(1==dir)
  {
    digitalWrite(IN1,1);
    digitalWrite(IN2,0);
    digitalWrite(ENA,1);
    delay(time);
  }
  else
  {
    digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    digitalWrite(ENA,1);
    delay(time);
  }
}
/*motor accelerate*/
void motorAcc(int dir,int time) {
  if (1==dir)
  {
    digitalWrite(IN1,1);
    digitalWrite(IN2,0);
    for (int i=0;i<=255;i++)
    {
      analogWrite(ENA,i);
      delay(time/255);
    } 
  }
  else
  {
    digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    for (int i=0;i<=255;i++)
    {
      analogWrite(ENA,i);
      delay(time/255);
    } 
  }
}

/*motor decelerate*/
void motorDec(int dir,int time) {
  if (1==dir)
  {
    digitalWrite(IN1,1);
    digitalWrite(IN2,0);
    for (int i=255;i>=0;i--)
    {
      analogWrite(ENA,i);
      delay(time/255);
    } 
  }
  else
  {
    digitalWrite(IN1,0);
    digitalWrite(IN2,1);
    for (int i=255;i>=0;i--)
    {
      analogWrite(ENA,i);
      delay(time/255);
    } 
  }
}
/*motor switch*/
void motorSwitch(int time)
{
  digitalWrite(IN1,0);
  digitalWrite(IN2,0);
  delay(time);
}

/*motor relax*/
void motorRelax(int time)
{
  digitalWrite(IN1,1);
  digitalWrite(IN2,1); 
  delay(time);
}

