#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<float, 4, 4> Matrix4f;

class FKMatrices
{
private:
  float J1_Lx = 0.1;
  float J1_Lz = 0.66;
  float J2_Lz = 0.3;
  float J3_Lz = 0.3;
  float J4_Lz = 0.1;
  float J5_Lx = 0.45;

public:
  FKMatrices()
  {
  }

  void fk(double J1_a, double J2_a, double J3_a, double J4_a, double J5_a, Matrix4f& FK)
  {
    FK(0, 0) = -sin(J1_a) * sin(J5_a) + cos(J1_a) * cos(J5_a) * cos(J2_a + J3_a + J4_a);
    FK(0, 1) = -sin(J2_a + J3_a + J4_a) * cos(J1_a);
    FK(0, 2) = -sin(J1_a) * cos(J5_a) - sin(J5_a) * cos(J1_a) * cos(J2_a + J3_a + J4_a);
    FK(0, 3) = J1_Lx * cos(J1_a) + J2_Lz * sin(J2_a) * cos(J1_a) + J3_Lz * sin(J2_a + J3_a) * cos(J1_a) +
               J4_Lz * sin(J2_a + J3_a + J4_a) * cos(J1_a) -
               J5_Lx * (sin(J1_a) * sin(J5_a) - cos(J1_a) * cos(J5_a) * cos(J2_a + J3_a + J4_a));
    FK(1, 0) = sin(J1_a) * cos(J5_a) * cos(J2_a + J3_a + J4_a) + sin(J5_a) * cos(J1_a);
    FK(1, 1) = -sin(J1_a) * sin(J2_a + J3_a + J4_a);
    FK(1, 2) = -sin(J1_a) * sin(J5_a) * cos(J2_a + J3_a + J4_a) + cos(J1_a) * cos(J5_a);
    FK(1, 3) = J1_Lx * sin(J1_a) + J2_Lz * sin(J1_a) * sin(J2_a) + J3_Lz * sin(J1_a) * sin(J2_a + J3_a) +
               J4_Lz * sin(J1_a) * sin(J2_a + J3_a + J4_a) +
               J5_Lx * (sin(J1_a) * cos(J5_a) * cos(J2_a + J3_a + J4_a) + sin(J5_a) * cos(J1_a));
    FK(2, 0) = -sin(J2_a + J3_a + J4_a) * cos(J5_a);
    FK(2, 1) = -cos(J2_a + J3_a + J4_a);
    FK(2, 2) = sin(J5_a) * sin(J2_a + J3_a + J4_a);
    FK(2, 3) = J1_Lz + J2_Lz * cos(J2_a) + J3_Lz * cos(J2_a + J3_a) + J4_Lz * cos(J2_a + J3_a + J4_a) -
               J5_Lx * sin(J2_a + J3_a + J4_a) * cos(J5_a);
  }

  void t1(double J1_a, double J2_a, double J3_a, double J4_a, double J5_a, Matrix4f& T1)
  {
    T1(0, 0) = cos(J1_a);
    T1(0, 1) = -sin(J1_a);
    T1(0, 2) = 0;
    T1(0, 3) = 0;
    T1(1, 0) = sin(J1_a);
    T1(1, 1) = cos(J1_a);
    T1(1, 2) = 0;
    T1(1, 3) = 0;
    T1(2, 0) = 0;
    T1(2, 1) = 0;
    T1(2, 2) = 1;
    T1(2, 3) = 0;
  }

  void t2(double J1_a, double J2_a, double J3_a, double J4_a, double J5_a, Matrix4f& T2)
  {
    T2(0, 0) = cos(J1_a) * cos(J2_a);
    T2(0, 1) = -sin(J1_a);
    T2(0, 2) = sin(J2_a) * cos(J1_a);
    T2(1, 3) = J1_Lx * cos(J1_a);
    T2(1, 0) = sin(J1_a) * cos(J2_a);
    T2(1, 1) = cos(J1_a);
    T2(1, 2) = sin(J1_a) * sin(J2_a);
    T2(2, 3) = J1_Lx * sin(J1_a);
    T2(2, 0) = -sin(J2_a);
    T2(2, 1) = 0;
    T2(2, 2) = cos(J2_a);
    T2(2, 3) = J1_Lz;
  }

  void t3(double J1_a, double J2_a, double J3_a, double J4_a, double J5_a, Matrix4f& T3)
  {
    T3(0, 0) = cos(J1_a) * cos(J2_a + J3_a);
    T3(0, 1) = -sin(J1_a);
    T3(0, 2) = sin(J2_a + J3_a) * cos(J1_a);
    T3(0, 3) = (J1_Lx + J2_Lz * sin(J2_a)) * cos(J1_a);
    T3(1, 0) = sin(J1_a) * cos(J2_a + J3_a);
    T3(1, 1) = cos(J1_a);
    T3(1, 2) = sin(J1_a) * sin(J2_a + J3_a);
    T3(1, 3) = (J1_Lx + J2_Lz * sin(J2_a)) * sin(J1_a);
    T3(2, 0) = -sin(J2_a + J3_a);
    T3(2, 1) = 0;
    T3(2, 2) = cos(J2_a + J3_a);
    T3(2, 3) = J1_Lz + J2_Lz * cos(J2_a);
  }

  void t4(double J1_a, double J2_a, double J3_a, double J4_a, double J5_a, Matrix4f& T4)
  {
    T4(0, 0) = cos(J1_a) * cos(J2_a + J3_a + J4_a);
    T4(0, 1) = -sin(J1_a);
    T4(0, 2) = sin(J2_a + J3_a + J4_a) * cos(J1_a);
    T4(0, 3) = (J1_Lx + J2_Lz * sin(J2_a) + J3_Lz * sin(J2_a + J3_a)) * cos(J1_a);
    T4(1, 0) = sin(J1_a) * cos(J2_a + J3_a + J4_a);
    T4(1, 1) = cos(J1_a);
    T4(1, 2) = sin(J1_a) * sin(J2_a + J3_a + J4_a);
    T4(1, 3) = (J1_Lx + J2_Lz * sin(J2_a) + J3_Lz * sin(J2_a + J3_a)) * sin(J1_a);
    T4(2, 0) = -sin(J2_a + J3_a + J4_a);
    T4(2, 1) = 0;
    T4(2, 2) = cos(J2_a + J3_a + J4_a);
    T4(2, 3) = J1_Lz + J2_Lz * cos(J2_a) + J3_Lz * cos(J2_a + J3_a);
  }

  void t5(double J1_a, double J2_a, double J3_a, double J4_a, double J5_a, Matrix4f& T5)
  {
    T5(0, 0) = -sin(J1_a) * sin(J5_a) + cos(J1_a) * cos(J5_a) * cos(J2_a + J3_a + J4_a);
    T5(0, 1) = -sin(J1_a) * cos(J5_a) - sin(J5_a) * cos(J1_a) * cos(J2_a + J3_a + J4_a);
    T5(0, 2) = sin(J2_a + J3_a + J4_a) * cos(J1_a);
    T5(0, 3) = (J1_Lx + J2_Lz * sin(J2_a) + J3_Lz * sin(J2_a + J3_a) + J4_Lz * sin(J2_a + J3_a + J4_a)) * cos(J1_a);
    T5(1, 0) = sin(J1_a) * cos(J5_a) * cos(J2_a + J3_a + J4_a) + sin(J5_a) * cos(J1_a);
    T5(1, 1) = -sin(J1_a) * sin(J5_a) * cos(J2_a + J3_a + J4_a) + cos(J1_a) * cos(J5_a);
    T5(1, 2) = sin(J1_a) * sin(J2_a + J3_a + J4_a);
    T5(1, 3) = (J1_Lx + J2_Lz * sin(J2_a) + J3_Lz * sin(J2_a + J3_a) + J4_Lz * sin(J2_a + J3_a + J4_a)) * sin(J1_a);
    T5(2, 0) = -sin(J2_a + J3_a + J4_a) * cos(J5_a);
    T5(2, 1) = sin(J5_a) * sin(J2_a + J3_a + J4_a);
    T5(2, 2) = cos(J2_a + J3_a + J4_a);
    T5(2, 3) = J1_Lz + J2_Lz * cos(J2_a) + J3_Lz * cos(J2_a + J3_a) + J4_Lz * cos(J2_a + J3_a + J4_a);
  }
};