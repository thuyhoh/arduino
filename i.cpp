#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
/* code robot dò line use PID
date_time:29-october-2023
author:Hung robocon*/
int in[4] = {8, 11, 12, 13};
int value[7];
int sensor[7] = {1, 2, 3, 4, 5, 6, 7};
// 0 1 2 3 4 5 6 // bắt đầu từ bên phải
#define ena 9
#define enb 10
void robot_tien();
void robot_retrai();
void robot_rephai();
void retrai90(int);
void rephai90(int);
void robot_dung();
float kp = 80; // 80-ổn định
float ki = 0;  //  có thể dẫn đến vọt lố
float kd = 30; // 30-ổn định
float error = 0, P = 0, I = 0, D = 0;
float sum_PID = 0, pre_error = 0;
int pwm_first = 200; // 150
int speed;
int dem = 0;
int count = 0;
int dem_ngang = 0;
bool check3 = true;
bool check = true;
bool check2 = true;
bool kiemtra = true;
unsigned long quakhu = 0;
unsigned long hientai;
int robocon = 1;
bool finish = true;
bool robot = true;
void setup()
{
    for (int i = 0; i < 4; i++)
    {
        pinMode(in[i], OUTPUT);
    }
    for (int i = 0; i < 7; i++)
    {
        pinMode(sensor[i], INPUT);
    }
    pinMode(ena, OUTPUT);
    pinMode(enb, OUTPUT);
    lcd.init();
    lcd.backlight();
}
void loop()
{
    lcd.setCursor(0, 0); // cột hàng
    lcd.print("ABU ROBOCON2024");
    read_sensor();
    tinh_pid();
    pwm_robot();
    if (error == 0)
    {
        robot_tien();
    }
    if (error == -1 || error == -2 || error == -3 || error == -3 || error == -4 || error == -5 || error == -6)
    {
        robot_rephai();
    }
    if (error == 1 || error == 2 || error == 3 || error == 4 || error == 5 || error == 6 || error == 7)
    {
        robot_retrai();
    }
}
void read_sensor()
{
    for (int i = 0; i < 7; i++)
    {
        value[i] = digitalRead(sensor[i]);
    }
    // phần 1:dò line thẳng và dò line tròn
    if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 0 && value[4] == 0 && value[5] == 0 && value[6] == 1)
    {
        error = 6;
    }
    if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 0 && value[4] == 0 && value[5] == 1 && value[6] == 0)
    {
        error = 5;
    }
    if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 0 && value[4] == 1 && value[5] == 0 && value[6] == 0)
    {
        error = 4;
    }
    if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 1 && value[4] == 1 && value[5] == 0 && value[6] == 0) /* cảm biến bên trái*/
    {
        error = 3;
    }
    if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 0 && value[4] == 1 && value[5] == 1 && value[6] == 0)
    {
        error = 2;
    }
    if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 0 && value[4] == 0 && value[5] == 1 && value[6] == 1)
    {
        error = 1;
    }
    if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 1 && value[4] == 0 && value[5] == 0 && value[6] == 0) // đi thẳng
    {
        error = 0;
    }
    if (value[0] == 0 && value[1] == 0 && value[2] == 1 && value[3] == 1 && value[4] == 0 && value[5] == 0 && value[6] == 0) // cảm biến bên phải
    {
        error = -1;
    }
    if (value[0] == 0 && value[1] == 1 && value[2] == 1 && value[3] == 0 && value[4] == 0 && value[5] == 0 && value[6] == 0)
    {
        error = -2;
    }
    if (value[0] == 1 && value[1] == 1 && value[2] == 0 && value[3] == 0 && value[4] == 0 && value[5] == 0 && value[6] == 0)
    {
        error = -3;
    }
    if (value[0] == 0 && value[1] == 0 && value[2] == 1 && value[3] == 0 && value[4] == 0 && value[5] == 0 && value[6] == 0)
    {
        error = -4;
    }
    if (value[0] == 0 && value[1] == 1 && value[2] == 0 && value[3] == 0 && value[4] == 0 && value[5] == 0 && value[6] == 0)
    {
        error = -5;
    }
    if (value[0] == 1 && value[1] == 0 && value[2] == 0 && value[3] == 0 && value[4] == 0 && value[5] == 0 && value[6] == 0)
    {
        error = -6;
    }
    // phần 2:rẽ góc vuông và đếm vị trí rẽ
    if(value[0] == 1 && value[1] == 1 && value[2] == 1 && value[3] == 1 && value[4] == 0 && value[5] == 0 && value[6] == 0 
    || value[0] == 1 && value[1] == 1 && value[2] == 1 && value[3] == 1 && value[4] == 1 && value[5] == 0 && value[6] == 0 
    || value[0] == 1 && value[1] == 1 && value[2] == 1 && value[3] == 1 && value[4] == 1 && value[5] == 1 && value[6] == 0 
    || value[0] == 0 && value[1] == 1 && value[2] == 1 && value[3] == 1 && value[4] == 1 && value[5] == 1 && value[6] == 1) // rẽ góc 90 bên phải
    {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             // thêm đoạn này nếu ko ổn
        if (check == true)
        {
            dem++;
            check = false;
        }
        if (dem == 1 || dem == 2 || dem == 7)
        {
            robot_dithang(200);
            delay(500);
            rephai90(200);
            delay(500);
            robot_dung();
            delay(150);
        }
        if (dem == 3)
        {
            retrai90(200);
            delay(150); // chú ý đoạn này khi thử sân chính
            robot_dung();
            delay(50);
        }
        if (dem == 4)
        {
            robot_dithang(200);
            delay(500);
            rephai90(200);
            delay(400);
            robot_dung();
            delay(150);
            // đoạn delay hết đoạn rẽ
            robot_dithang(200);
            delay(1500); // chỉnh ở đây
            retrai90(200);
            delay(300); // chỉnh lại
            robot_dung();
            delay(150);
        }
        if (dem == 5)
        {
            robot_dithang(200);
            delay(500);
            rephai90(200);
            delay(500);
            robot_dung();
            delay(150);
        }
        if (dem == 6)
        {
            robot_dithang(200);
            delay(500);
            rephai90(200);
            delay(400);
            robot_dung();
            delay(100);
            robot_dithang(200);
            delay(1600);
            rephai90(200);
            delay(400);
            robot_dithang(200);
            delay(600);
            rephai90(200);
            delay(200);
            robot_dung();
            delay(100);
            finish = true;
        }
        if (dem == 8)
        {
            robot_dithang(200);
            delay(250);
            rephai90(200);
            delay(400);
            robot_dung();
            delay(100);
        }
    }
    else
    {
        check = true;
    }
    if (value[0] == 0 && value[1] == 0 && value[2] == 0 && value[3] == 1 && value[4] == 1 && value[5] == 1 && value[6] == 1) // rẽ góc 90 bên trái
    {
        robot_dithang(200);
        delay(500);
        retrai90(200);
        delay(500);
        robot_dung();
        delay(150);
    }
    // rẽ vạch line full ngang
    if (value[0] == 1 && value[1] == 1 && value[2] == 1 && value[3] == 1 && value[4] == 1 && value[5] == 1 && value[6] == 1    // rẽ vạch line full ngang
        || value[0] == 0 && value[1] == 1 && value[2] == 1 && value[3] == 1 && value[4] == 1 && value[5] == 1 && value[6] == 0 // 2
        || value[0] == 1 && value[1] == 0 && value[2] == 1 && value[3] == 1 && value[4] == 1 && value[5] == 1 && value[6] == 1 // 6
        || value[0] == 1 && value[1] == 0 && value[2] == 0 && value[3] == 1 && value[4] == 0 && value[5] == 1 && value[6] == 1)
    { // 9
        // 1   0   0   1   0   1   1
        if (check3 == true)
        {
            dem_ngang++;
            check3 = false;
        }
        if (dem_ngang == 1)
        {
            robot_dithang(200);
            delay(500);
            retrai90(200);
            delay(500);
            robot_dung();
            delay(100);
            finish = false;
        }
        if (dem_ngang == 2)
        {
            robot_dithang(200);
            delay(500);
            rephai90(200);
            delay(400);
            robot_dung();
            delay(100);
            finish = true;
        }
    }
    else
    {
        check3 = true;
    }
    // phần 3 : rẽ góc nhọn và đếm vị trí rẽ
    if (value[0] == 0 && value[1] == 0 && value[2] == 1 && value[3] == 1 && value[4] == 1 && value[5] == 0 && value[6] == 0 && finish == true)
    {
        if (check2 == true)
        {
            count++;
            check2 = false;
        }
        if (count == 1)
        {
            retrai90(200);
            delay(200);
            robot_dung();
            delay(50);
        }
        if (count == 2)
        {
            rephai90(200);
            delay(180);
            robot_dung();
            delay(100);
        }
    }
    else
    {
        check2 = true;
    }
    // lưu ý khi bị lỗi cần tắt nguồn và bật lại để biến đếm có thể đếm lại từ đầu nếu ko robot sẽ bị lỗi
}
void tinh_pid()
{
    P = error;
    I = I + error;         // i bằng tổng các sai số quá khứ
    D = error - pre_error; // sai số hiện tại trừ sai số quá khứ

    sum_PID = (kp * P) + (ki * I) + (kd * D);

    pre_error = error; // gán sai số quá khứ bằng sai số hiện tại
}
void pwm_robot()
{
    int left_robot  = pwm_first - sum_PID;
    int right_robot = pwm_first + sum_PID;

    left_robot = constrain(left_robot, 0, 255); // hàm constrain sẽ giới hạn giá trị nằm trong khoảng 0-255
    right_robot = constrain(right_robot, 0, 255);

    analogWrite(ena, left_robot);
    analogWrite(enb, right_robot); // chỉnh lại nếu có bị sai
}
void robot_tien()
{
    digitalWrite(in[0], HIGH);
    digitalWrite(in[1], LOW);

    digitalWrite(in[2], HIGH);
    digitalWrite(in[3], LOW);
}
void robot_dung()
{
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(in[i], LOW);
    }
}
void robot_lui()
{
    digitalWrite(in[0], LOW);
    digitalWrite(in[1], HIGH);
    digitalWrite(in[2], LOW);
    digitalWrite(in[3], HIGH);
}
void robot_retrai()
{
    digitalWrite(in[0], LOW);
    digitalWrite(in[1], HIGH);

    digitalWrite(in[2], HIGH);
    digitalWrite(in[3], LOW);
}
void robot_rephai()
{
    digitalWrite(in[0], HIGH);
    digitalWrite(in[1], LOW);
    digitalWrite(in[2], LOW);
    digitalWrite(in[3], HIGH);
}
void retrai90(int speed)
{
    analogWrite(ena, speed);
    digitalWrite(in[0], LOW);
    digitalWrite(in[1], HIGH);
    analogWrite(enb, speed);
    digitalWrite(in[2], HIGH);
    digitalWrite(in[3], LOW);
}
void rephai90(int speed)
{
    analogWrite(ena, speed);
    digitalWrite(in[0], HIGH);
    digitalWrite(in[1], LOW);
    digitalWrite(in[2], LOW);
    analogWrite(enb, speed);
    digitalWrite(in[3], HIGH);
}
void robot_dithang(int speed)
{
    analogWrite(ena, speed);
    digitalWrite(in[0], HIGH);
    digitalWrite(in[1], LOW);
    analogWrite(enb, speed);
    digitalWrite(in[2], HIGH);
    digitalWrite(in[3], LOW);
}