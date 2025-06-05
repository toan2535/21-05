#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif
#include "ros.h"
#include "ros/time.h"
// Header file cho việc publish vận tốc cho odom
#include "lino_msgs/Velocities.h"
// Header file cho việc subscribe cmd_vel
#include "geometry_msgs/Twist.h"
// Header file cho PID server
#include "lino_msgs/PID.h"

#include "lino_base_config.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"

#define ENCODER_OPTIMIZE_INTERRUPTS // Comment this out on Non-Teensy boards
#include "Encoder.h"

#define COMMAND_RATE 20 // Hz
#define DEBUG_RATE 5

#define WHEEL_BASE 0.3 // Adjust to your robot's wheelbase in meters

// Chỉ sử dụng 2 encoder cho 2 bánh
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV);

// Chỉ sử dụng 2 động cơ
Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Cấu hình kinematics cho Differential Drive
Kinematics kinematics(Kinematics::DIFFERENTIAL_DRIVE, MAX_RPM, WHEEL_DIAMETER, 0, WHEEL_BASE); // WHEEL_BASE là khoảng cách giữa 2 bánh

float g_req_linear_vel_x = 0;
float g_req_angular_vel_z = 0;
unsigned long g_prev_command_time = 0;

// Callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const lino_msgs::PID& pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void setup()
{
    // Thiết lập các chân bổ sung ở mức HIGH
    pinMode(EXTRA_PIN_1, OUTPUT);
    pinMode(EXTRA_PIN_2, OUTPUT);
    pinMode(EXTRA_PIN_3, OUTPUT);
    pinMode(EXTRA_PIN_4, OUTPUT);

    digitalWrite(EXTRA_PIN_1, HIGH); // Đặt chân 4 ở mức 5V
    digitalWrite(EXTRA_PIN_2, HIGH); // Đặt chân 5 ở mức 5V
    digitalWrite(EXTRA_PIN_3, HIGH); // Đặt chân 6 ở mức 5V
    digitalWrite(EXTRA_PIN_4, HIGH); // Đặt chân 14 ở mức 5V

    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("LINOBASE CONNECTED");
    delay(1);
}

void loop()
{
    static unsigned long prev_control_time = 0;
    static unsigned long prev_debug_time = 0;

    // Điều khiển robot dựa trên tần số định nghĩa
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        moveBase();
        prev_control_time = millis();
    }

    // Dừng robot khi không nhận được lệnh trong 200ms
    if ((millis() - g_prev_command_time) >= 200)
    {
        stopBase();
    }

    // In thông tin debug nếu cần
    if (DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }

    nh.spinOnce();
}

void PIDCallback(const lino_msgs::PID& pid)
{
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_angular_vel_z = cmd_msg.angular.z;
    g_prev_command_time = millis();
}

void moveBase()
{
    // Tính RPM yêu cầu cho mỗi động cơ
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, 0, g_req_angular_vel_z);

    // Lấy tốc độ hiện tại của mỗi động cơ
    int current_rpm1 = motor1_encoder.getRPM();
    int current_rpm2 = motor2_encoder.getRPM();

    // Tính PWM bằng PID
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1)); // Sửa lỗi: sử dụng motor1_pid và req_rpm.motor1
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));

    // Tính vận tốc hiện tại
    Kinematics::velocities current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, 0, 0);

    // Publish vận tốc
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_angular_vel_z = 0;
}

void printDebug()
{
    char buffer[50];
    sprintf(buffer, "Encoder Left : %ld", motor1_encoder.read());
    nh.loginfo(buffer);
    sprintf(buffer, "Encoder Right: %ld", motor2_encoder.read());
    nh.loginfo(buffer);
}