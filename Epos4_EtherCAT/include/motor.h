#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>

struct Motor {
    int ID;
    int cnt;
    int N; // encoder ppm, quadrature
    int gearratio;
    int degree;
    int Min, Max;
    int homeposition;
};

void MotorID(struct Motor *motor, int id) {
    motor->ID = id;
}

void SetEncodorandGear(struct Motor *motor, int encodor, float gearRatio) {
    motor->N = encodor * 4;
    motor->gearratio = gearRatio;
}

void EncodorHomeposition(struct Motor *motor, int home) {
    motor->homeposition = home;
}

void SetDegree(struct Motor *motor, int jointdegree) {
    motor->degree = jointdegree;
}

void SetJointMinMiax(struct Motor *motor, int min, int max) {
    motor->Min = min;
    motor->Max = max;
}

int MotorOption(const struct Motor *motor) {
    if (motor->Min > motor->degree || motor->Max < motor->degree) {
        printf("모터 %d out of range\n", motor->ID);
        return 0;
    } else {
        return 1;
    }
}

int GetEncoderCnt(struct Motor *motor) {
    motor->cnt = motor->degree * motor->N * motor->gearratio / 360 + motor->homeposition;
    return motor->cnt;
}

void ShowCNT(const struct Motor *motor) {
    printf("엔코더에 카운트된 펄스 수 %d\n", motor->cnt);
}

#endif