#include <Arduino.h>
#include "Emakefun_MotorDriver.h"
#include "PS2X_lib.h"
#include <PID_v1.h>

uint8_t speed_val = 150;  // set (tốc độ của robot) speed_robot (pwm value), 0 < speed_robot < 256
int8_t check_out = 0;

/// define sensor pinout
#define line_1      A0 // trái (hoặc ngược lại)
#define line_2      A1
#define line_3      A2 // giữa
#define line_4      A3 
#define line_5      A4 // phải
// define ps2
#define PS2_DAT     12
#define PS2_CMD     11
#define PS2_SEL     10
#define PS2_CLK     13

#define pressures   false
#define rumble      false
//Define Variables we'll be connecting to
double Setpoint = 0, Input, Output;
uint8_t flag_zero = 0;

//double Kp=15, Ki=0.045, Kd=12;  
double Kp=20, Ki=0.04555555, Kd=11.898989; 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PS2X ps2x;
uint8_t sensor;

Emakefun_MotorDriver mMotor = Emakefun_MotorDriver(0x60);
Emakefun_DCMotor *DCMotor_1 = mMotor.getMotor(1);
Emakefun_DCMotor *DCMotor_2 = mMotor.getMotor(2);
Emakefun_DCMotor *DCMotor_3 = mMotor.getMotor(3);
Emakefun_DCMotor *DCMotor_4 = mMotor.getMotor(4);

void setMotorSpeed(Emakefun_DCMotor* motor, int speed, int direction) {
    motor->setSpeed(speed);
    motor->run(direction);
}

void moveForward();
void moveBackward();
void strafeLeft();
void strafeRight();
void rotateLeft();
void rotateRight();
void stopMotors();
void diagonalFrontRight();
void diagonalBackLeft();
void diagonalFrontLeft();
void diagonalBackRight();

void setup() {
    Serial.begin(9600);
    mMotor.begin(50);
    ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    pinMode(line_1, INPUT);
    pinMode(line_2, INPUT);
    pinMode(line_3, INPUT);
    pinMode(line_4, INPUT);
    pinMode(line_5, INPUT);

    Input = 0; // mặc định line giữa =0, => luôn luôn điều chỉnh xe để input=0
    myPID.SetSampleTime(1); // thời gian lấy mẫu phụ thuộc tốc độ xe, lấy mẫu càng nhanh càng tốt
    myPID.SetMode(AUTOMATIC); 
    myPID.SetOutputLimits(-speed_val, speed_val); // giá trị tốc độ, -speed tức bánh bên trái quay max, bên phải ngừng quay

}

///// băm xung điều khiển tốc độ 2 bánh xe, từ đó ta điều khiển được hướng rẽ, tốc dộ..... thông qua 1 biến duy nhất
void motorControl(int16_t duty_value) {
    int16_t speed_a, speed_b;
    int speed_zero;
    speed_zero = speed_val / 2;
    if(duty_value > 1) {
        speed_b = -speed_zero;
        speed_a = duty_value;
    }
    else if(duty_value == 0) {
        speed_a = speed_b = 0;
    }
    else if(duty_value < -1) {
        speed_a = -speed_zero;
        speed_b = -duty_value;
    } 
    setMotorSpeed(DCMotor_1, speed_b + speed_zero, FORWARD);
    setMotorSpeed(DCMotor_2, speed_b + speed_zero, FORWARD);
    setMotorSpeed(DCMotor_3, speed_a + speed_zero, FORWARD);
    setMotorSpeed(DCMotor_4, speed_a + speed_zero, FORWARD);
}     

//Sáng đèn là 0v, đèn tắt (vào line đen) là 5V, từ 5 cảm biến ta có 9 vị trí -4 -3 -2 -1 0 1 2 3 4
void scan_sensor() {
    if(digitalRead(line_5)==1) {
      sensor = 4;
    }
    else if((digitalRead(line_4) == 1)&&(digitalRead(line_5) == 1)) {
      sensor = 3;
    }
    else if(digitalRead(line_4) == 1) {
      sensor = 2;
    }
    else if((digitalRead(line_3) == 1)&&(digitalRead(line_4) == 1)) {
      sensor = 1;
    }
    else if(digitalRead(line_3) == 1) {
      sensor = 0;
    }
    else if((digitalRead(line_2) == 1)&&(digitalRead(line_3) == 1)) {
      sensor = -1;
    }
    else if(digitalRead(line_2) == 1) {
      sensor = -2;
    }
    else if((digitalRead(line_1) == 1)&&(digitalRead(line_2) == 1)) {
      sensor = -3;
    }
    else if(digitalRead(line_1) == 1) {
      sensor = -4;
    }
}

void loop() {
    // Setpoint = 0;
    // scan_sensor();
    // Input = sensor;
    // myPID.Compute();
    // motorControl(Output);
    if (!ps2x.NewButtonState()) {
        if (ps2x.Button(PSB_PAD_UP)) moveForward();
        if (ps2x.Button(PSB_PAD_DOWN)) moveBackward();
        if (ps2x.Button(PSB_PAD_LEFT)) strafeLeft();
        if (ps2x.Button(PSB_PAD_RIGHT)) strafeRight();
        if (ps2x.ButtonPressed(PSB_CIRCLE)) rotateLeft();
        if (ps2x.ButtonPressed(PSB_CROSS)) stopMotors();
        if (ps2x.ButtonPressed(PSB_SQUARE)) rotateRight();
        if (ps2x.ButtonPressed(PSB_TRIANGLE)) diagonalFrontRight();
    } else {
        stopMotors();
    }
}

void moveForward() {
    setMotorSpeed(DCMotor_1, speed_val, FORWARD);
    setMotorSpeed(DCMotor_2, speed_val, FORWARD);
    setMotorSpeed(DCMotor_3, speed_val, FORWARD);
    setMotorSpeed(DCMotor_4, speed_val, FORWARD);
}

void moveBackward() {
    setMotorSpeed(DCMotor_1, speed_val, BACKWARD);
    setMotorSpeed(DCMotor_2, speed_val, BACKWARD);
    setMotorSpeed(DCMotor_3, speed_val, BACKWARD);
    setMotorSpeed(DCMotor_4, speed_val, BACKWARD);
}

void strafeLeft() {
    setMotorSpeed(DCMotor_1, speed_val, BACKWARD);
    setMotorSpeed(DCMotor_2, speed_val, FORWARD);
    setMotorSpeed(DCMotor_3, speed_val, FORWARD);
    setMotorSpeed(DCMotor_4, speed_val, BACKWARD);
}

void strafeRight() {
    setMotorSpeed(DCMotor_1, speed_val, FORWARD);
    setMotorSpeed(DCMotor_2, speed_val, BACKWARD);
    setMotorSpeed(DCMotor_3, speed_val, BACKWARD);
    setMotorSpeed(DCMotor_4, speed_val, FORWARD);
}

void rotateLeft() {
    setMotorSpeed(DCMotor_1, speed_val, FORWARD);
    setMotorSpeed(DCMotor_2, speed_val, BACKWARD);
    setMotorSpeed(DCMotor_3, speed_val, FORWARD);
    setMotorSpeed(DCMotor_4, speed_val, BACKWARD);
}

void rotateRight() {
    setMotorSpeed(DCMotor_1, speed_val, BACKWARD);
    setMotorSpeed(DCMotor_2, speed_val, FORWARD);
    setMotorSpeed(DCMotor_3, speed_val, BACKWARD);
    setMotorSpeed(DCMotor_4, speed_val, FORWARD);
}

void stopMotors() {
    DCMotor_1->run(BRAKE);
    DCMotor_2->run(BRAKE);
    DCMotor_3->run(BRAKE);
    DCMotor_4->run(BRAKE);
}

void diagonalFrontRight() {
    setMotorSpeed(DCMotor_1, speed_val, FORWARD);
    DCMotor_2->run(BRAKE);
    DCMotor_3->run(BRAKE);
    setMotorSpeed(DCMotor_4, speed_val, FORWARD);
}

void diagonalBackLeft() {
    setMotorSpeed(DCMotor_1, speed_val, BACKWARD);
    DCMotor_2->run(BRAKE);
    DCMotor_3->run(BRAKE);
    setMotorSpeed(DCMotor_4, speed_val, BACKWARD);
}

void diagonalFrontLeft() {
    setMotorSpeed(DCMotor_2, speed_val, FORWARD);
    DCMotor_1->run(BRAKE);
    DCMotor_4->run(BRAKE);
    setMotorSpeed(DCMotor_3, speed_val, FORWARD);
}

void diagonalBackRight() {
    setMotorSpeed(DCMotor_2, speed_val, BACKWARD);
    DCMotor_1->run(BRAKE);
    DCMotor_4->run(BRAKE);
    setMotorSpeed(DCMotor_3, speed_val, BACKWARD);
}
