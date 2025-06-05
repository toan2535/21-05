#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

// Bỏ chú thích cho loại robot bạn đang xây dựng
#define LINO_BASE DIFFERENTIAL_DRIVE // Robot 2WD hoặc robot bánh xích với 2 động cơ
// #define LINO_BASE SKID_STEER      // Robot 4WD
// #define LINO_BASE ACKERMANN       // Robot điều hướng kiểu xe hơi với 2 động cơ
// #define LINO_BASE ACKERMANN1      // Robot điều hướng kiểu xe hơi với 1 động cơ
// #define LINO_BASE MECANUM         // Robot điều khiển Mecanum

// Bỏ chú thích cho driver động cơ bạn đang sử dụng
// #define USE_L298_DRIVER
#define USE_BTS7960_DRIVER
// #define USE_ESC

// Bỏ chú thích cho IMU bạn đang sử dụng
// #define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

#define DEBUG 0

//=================THÔNG SỐ ROBOT LỚN HƠN (BTS7960)=============================
#define K_P 0.15  // Hằng số P
#define K_I 0.37   // Hằng số I
#define K_D 0.01   // Hằng số D

// Xác định thông số robot của bạn tại đây
#define MAX_RPM 220               // Tốc độ tối đa của động cơ (RPM)
#define COUNTS_PER_REV 3467      // Số xung của bộ mã hóa bánh xe mỗi vòng
#define WHEEL_DIAMETER 0.1       // Đường kính bánh xe (mét)
#define PWM_BITS 8               // Độ phân giải PWM của vi điều khiển
#define LR_WHEELS_DISTANCE 0.32  // Khoảng cách giữa bánh trái và phải
// #define FR_WHEELS_DISTANCE 0.38  // Khoảng cách giữa bánh trước và sau. Bỏ qua nếu dùng 2WD/ACKERMANN
//=================KẾT THÚC THÔNG SỐ ROBOT LỚN HƠN =============================

/*
HƯỚNG ROBOT
         PHÍA TRƯỚC
    ĐỘNG CƠ 1  ĐỘNG CƠ 2  (2WD/ACKERMANN)
    ĐỘNG CƠ 3  ĐỘNG CƠ 4  (4WD/MECANUM)  
         PHÍA SAU
*/

/// CHÂN BỘ MÃ HÓA
#define MOTOR1_ENCODER_A 8
#define MOTOR1_ENCODER_B 2

#define MOTOR2_ENCODER_A 3
#define MOTOR2_ENCODER_B 9 

// CHÂN ĐỘNG CƠ
#ifdef USE_BTS7960_DRIVER
  #define MOTOR_DRIVER BTS7960  

  #define MOTOR1_PWM 1 // Giá trị giữ chỗ, không sử dụng cho BTS7960
  #define MOTOR1_IN_A 11 // L_PWM cho Động cơ 1
  #define MOTOR1_IN_B 10 // R_PWM cho Động cơ 1

  #define MOTOR2_PWM 0 // Giá trị giữ chỗ, không sử dụng cho BTS7960
  #define MOTOR2_IN_A 13  // L_PWM cho Động cơ 2
  #define MOTOR2_IN_B 12  // R_PWM cho Động cơ 2

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

// CHÂN BỔ SUNG
#define EXTRA_PIN_1 4 // Chân bổ sung 1
#define EXTRA_PIN_2 5 // Chân bổ sung 2
#define EXTRA_PIN_3 6 // Chân bổ sung 3
#define EXTRA_PIN_4 7 // Chân bổ sung 4

#define STEERING_PIN 20

#endif