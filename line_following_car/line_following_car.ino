#include <Servo.h>

// motor
int ENA = 6;
int ENB = 5;
const int in[4] = {7,8,3,4}; 

// line sensor
int s1 = A1; int l1;
int s2 = A2; int l2;
int s3 = A3; int l3;
int s4 = A4; int l4;
int s5 = A5; int l5;
// servo

// PID
// error value and PID
int error; int previous_error = 0;
int P; int Kp = 12;
int I; int Ki = 10;
int D; int Kd =10;
int PID_value;

int left_motor_speed(0,255); int initial_motor_speed = 150;
int right_motor_speed(0,255);


void setup()
{
  pinMode(s1 , INPUT);
  pinMode(s2 , INPUT);
  pinMode(s3 , INPUT);
  pinMode(s4 , INPUT);
  pinMode(s5 , INPUT);
  Serial.begin(9600);
}
void loop()
{
  errorCalculation();
  pid_calculation();
  speedControl();
  MotorControl();
}

void MotorControl()
{
  if()
}


//calculation error by sensor detech situation
void errorCalculation(){
  l1 = digitalRead(s1);
  l2 = digitalRead(s2);
  l3 = digitalRead(s3);
  l4 = digitalRead(s4);
  l5 = digitalRead(s5);
  if (l5 == 1 &&l4 == 1 &&l3 == 1 &&l2 == 1 &&l1 == 0){error =4;} //1
  if (l5 == 1 &&l4 == 1 &&l3 == 1 &&l2 == 0 &&l1 == 0){error =3;} //2
  if (l5 == 1 &&l4 == 1 &&l3 == 1 &&l2 == 0 &&l1 == 1){error =2;} //3
  if (l5 == 1 &&l4 == 1 &&l3 == 0 &&l2 == 0 &&l1 == 1){error =1;} //4
  if (l5 == 1 &&l4 == 1 &&l3 == 0 &&l2 == 1 &&l1 == 1){error =0;} //5
  if (l5 == 1 &&l4 == 0 &&l3 == 0 &&l2 == 1 &&l1 == 1){error =-1;} //6
  if (l5 == 1 &&l4 == 0 &&l3 == 1 &&l2 == 1 &&l1 == 1){error =-2;} //7
  if (l5 == 0 &&l4 == 0 &&l3 == 1 &&l2 == 1 &&l1 == 1){error =-3;} //8
  if (l5 == 0 &&l4 == 1 &&l3 == 1 &&l2 == 1 &&l1 == 1){error =-4;} //9
  if (l5 == 0 &&l4 == 0 &&l3 == 0 &&l2 == 0 &&l1 == 0){error =-5;} //10

//Serial.print(l1);Serial.print("  ");Serial.print(l2);Serial.print("  ");Serial.print(l3);Serial.print("  ");
//Serial.print(l4);Serial.print("  "); Serial.print(l5);Serial.println("  ");Serial.print("error");Serial.print(error);Serial.println("  ");Serial.println("  ");
}

void pid_calculation(){
  P = error;
  I = I + error;
  D = D - previous_error;
  PID_value = Kp*P + Ki*I + Kd*D;
  previous_error = error;
  //Serial.print("PID: ");Serial.println(PID_value);
}

void speedControl(){
  left_motor_speed = initial_motor_speed - PID_value;
  right_motor_speed = initial_motor_speed + PID_value;
  analogWrite(ENA,left_motor_speed);
  analogWrite(ENA,left_motor_speed);
}

void robot_tien()
{ 
  digitalWrite(in[0],HIGH);
  digitalWrite(in[1],LOW);
 
  digitalWrite(in[2],HIGH);
  digitalWrite(in[3],LOW);
}

void robot_dung()
{
  for(int i=0;i<4;i++)
  {
    digitalWrite(in[i],LOW);
  }
}

void robot_retrai()
{
  digitalWrite(in[0],LOW);
  digitalWrite(in[1],HIGH);
 
  digitalWrite(in[2],HIGH);
  digitalWrite(in[3],LOW);
}
void robot_rephai()
{
  digitalWrite(in[0],HIGH);
  digitalWrite(in[1],LOW);
  digitalWrite(in[2],LOW);
  digitalWrite(in[3],HIGH);
}
void retrai90(int speed)
{
  analogWrite(ena,speed);
  digitalWrite(in[0],LOW);
  digitalWrite(in[1],HIGH);
  analogWrite(enb,speed);
  digitalWrite(in[2],HIGH);
  digitalWrite(in[3],LOW);
}
void rephai90(int speed)
{
  analogWrite(ena,speed);
  digitalWrite(in[0],HIGH);
  digitalWrite(in[1],LOW);
  digitalWrite(in[2],LOW);
  analogWrite(enb,speed);
  digitalWrite(in[3],HIGH);
}
void robot_dithang(int speed)
{
  analogWrite(ena,speed);
   digitalWrite(in[0],HIGH);
  digitalWrite(in[1],LOW);
 analogWrite(enb,speed);
  digitalWrite(in[2],HIGH);
  digitalWrite(in[3],LOW);
}

