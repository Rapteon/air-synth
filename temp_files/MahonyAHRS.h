#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#include <Eigen/Dense>

class MahonyAHRS {
private:
    float twoKp;
    float twoKi;
    float q0, q1, q2, q3;
    float integralFBx, integralFBy, integralFBz;

    float invSqrt(float x);

public:
        float samplePeriod; // this needs to be consistent with sampleFreq
    MahonyAHRS(float kp, float ki)
        : twoKp(2.0f * kp), twoKi(2.0f * ki), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
          integralFBx(0.0f), integralFBy(0.0f), integralFBz(0.0f) {}

    void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    Eigen::Matrix3f getRotationMatrix() const {
        Eigen::Matrix3f R;
        R << 1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2),
             2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1),
             2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2);
        return R;
    }

    float getQ0() const { return q0; }
    float getQ1() const { return q1; }
    float getQ2() const { return q2; }
    float getQ3() const { return q3; }
};

#endif

