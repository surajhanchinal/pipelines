#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <utility>
#include <iostream>
#include "fk_matrices.h"
#include <algorithm>
#include "trajectory_store.h"
typedef Eigen::Matrix<float, 4, 8> Cuboid;

struct Solution{
    double j1,j2,j3,j4,j5;
};

// e suffix stands for estimate
class IKSolver
{
    public:

  const float J1_Lx = 0.0525;
  const float J1_Lz = 0.66;
  const float J2_Lz = 0.3;
  const float J3_Lz = 0.3;
  const float J4_Lz = 0.1;
  const float J5_Lx = 0.45;
  FKMatrices* fkMatrices;
  Matrix4f FK;
  Matrix4f T1,T2,T3,T4,T5;
  std::vector<Cuboid> cuboids;
  //std::vector<std::pair<int,int>> collisionPairs = {{0,2},{0,3},{0,4},{1,3},{1,4},{2,4}};
  std::vector<std::pair<int,int>> collisionPairs = {{0,2},{0,3},{0,4},{1,3},{1,4},{2,4}};
  Cuboid cb1,cb2,cb3,cb4,cb5;
  

  float c1_1(float j1e, float j5e, float j234e, float Px)
  {
    return J1_Lx + J4_Lz * sin(j234e) - J5_Lx * sin(j5e) * tan(j1e) + J5_Lx * cos(j5e) * cos(j234e) - Px / cos(j1e);
  }

  float c1_2(float j1e, float j5e, float j234e, float Py)
  {
    return J1_Lx + J4_Lz * sin(j234e) + J5_Lx * sin(j5e) / tan(j1e) + J5_Lx * cos(j5e) * cos(j234e) - Py / sin(j1e);
  }

  float c1(float j1e, float j5e, float j234e, float Px, float Py)
  {
    if (abs(j1e) - M_PI_2 < 0.05)
    {
      return c1_1(j1e, j5e, j234e, Px);
    }
    return c1_2(j1e, j5e, j234e, Py);
  }

  float c2(float j5e, float j234e, float Pz)
  {
    return J1_Lz + J4_Lz * cos(j234e) - J5_Lx * sin(j234e) * cos(j5e) - Pz;
  }

  float k1()
  {
    return J2_Lz;
  }

  float k2()
  {
    return J3_Lz;
  }

  /*
    k1*cos(j2) + k2*cos(j2+j3) + c2 = 0 ## x
    k1*sin(j2) + k2*sin(j2+j3) + c1 = 0 ## y
  */

  float j3(float j1e, float j5e, float j234e, float Px, float Py, float Pz)
  {
    float c1_e = c1(j1e, j5e, j234e, Px, Py);
    float c2_e = c2(j5e, j234e, Pz);
    float k1_e = k1();
    float k2_e = k2();
    return acos((c1_e * c1_e + c2_e * c2_e - k1_e * k1_e - k2_e * k2_e) / 2 * k1_e * k2_e);
  }

  std::pair<float, float> j2(float j3e, float j1e, float j5e, float j234e, float Px, float Py, float Pz)
  {
    float c1_e = c1(j1e, j5e, j234e, Px, Py);
    float c2_e = c2(j5e, j234e, Pz);
    float k1_e = k1();
    float k2_e = k2();
    float t1 = atan(c1_e / c2_e);
    float t2 = atan(k2_e * sin(j3e) / (k1_e + k2_e * cos(j3e)));
    float sol1 = t1 - t2;
    float sol2 = t1 + t2;
    return { sol1, sol2 };
  }

  float j1(float tyy, float txy)
  {
    return atan(tyy / txy);
  }

  float j5(float tzz, float tzx)
  {
    return -atan(tzz / tzx);
  }

  float j5_new(float j1, float Px, float Py)
  {
    return asin((-Px * sin(j1) + Py * cos(j1)) / J5_Lx);
  }

  float j234(float tzx, float tzy, float j5e)
  {
    return atan(tzx / (tzy * cos(j5e)));
  }

  bool checkFullSolution(float j1e,float j2e,float j3e,float j4e,float j5e,Matrix4f &pose_matrix){
    fkMatrices->fk(j1e,j2e,j3e,j4e,j5e,FK);
    return ((FK - pose_matrix).array().abs() < 0.01).all();
  }

  bool checkPositionSolution(float j1e,float j2e,float j3e,float j4e,float j5e,float Px,float Py,float Pz){
    fkMatrices->fk(j1e,j2e,j3e,j4e,j5e,FK);
    bool isValid = abs(FK(0,3) - Px) < 0.01 and abs(FK(1,3) - Py) < 0.01 and abs(FK(2,3) - Pz) < 0.01;
    if(isValid){
      bool collided = hasCollision(j1e,j2e,j3e,j4e,j5e);
      //std::cout<<"zero error, collision: "<<collided<<std::endl;
      if(!collided){
        return true;
      }
    }
    return false;
  }

  bool hasGap(Eigen::Vector4f &ax,Cuboid &cb1,Cuboid cb2){
    float cb1_min = 100000;
    float cb1_max=  -100000;
    float cb2_min = 100000;
    float cb2_max=  -100000;

    for(int i=0;i<8;i++){
      float v1 = cb1.col(i).dot(ax);
      float v2 = cb2.col(i).dot(ax);
      cb1_min = std::min(v1,cb1_min);
      cb1_max = std::max(v1,cb1_max);

      cb2_min = std::min(v2,cb2_min);
      cb2_max = std::max(v2,cb2_max);
    }

    if(cb2_max >= cb1_min and cb2_min <= cb1_max){
      return false;
    }
    else{
      return true;
    }
  }


  bool hasCollision(float j1e,float j2e,float j3e,float j4e,float j5e){
    fkMatrices->t1(j1e,j2e,j3e,j4e,j5e,T1);
    fkMatrices->t2(j1e,j2e,j3e,j4e,j5e,T2);
    fkMatrices->t3(j1e,j2e,j3e,j4e,j5e,T3);
    fkMatrices->t4(j1e,j2e,j3e,j4e,j5e,T4);
    fkMatrices->t5(j1e,j2e,j3e,j4e,j5e,T5);

    initializeCubes();
    Cuboid &cb1 = cuboids[0];
    Cuboid &cb2 = cuboids[1];
    Cuboid &cb3 = cuboids[2];
    Cuboid &cb4 = cuboids[3];
    Cuboid &cb5 = cuboids[4];
    cb1 = T1*cb1;
    cb2 = T2*cb2;
    cb3 = T3*cb3;
    cb4 = T4*cb4;
    cb5 = T5*cb5;
    std::vector<Eigen::Vector4f> axes(15);

    bool collision = false;

    for(auto &cp : collisionPairs){
      Cuboid &cb1 = cuboids[cp.first];
      Cuboid &cb2 = cuboids[cp.second];
      // cube1 axes
      axes[0] = cb1.col(0) - cb1.col(1);
      axes[1] = cb1.col(0) - cb1.col(3);
      axes[2] = cb1.col(0) - cb1.col(4);
      //cube2 axes
      axes[3] = cb2.col(0) - cb2.col(1);
      axes[4] = cb2.col(0) - cb2.col(3);
      axes[5] = cb2.col(0) - cb2.col(4);
      // cross product axes
      axes[6] = axes[0].cross3(axes[3]);
      axes[7] = axes[0].cross3(axes[4]);
      axes[8] = axes[0].cross3(axes[5]);

      axes[9] = axes[1].cross3(axes[3]);
      axes[10] = axes[1].cross3(axes[4]);
      axes[11] = axes[1].cross3(axes[5]);

      axes[12] = axes[2].cross3(axes[3]);
      axes[13] = axes[2].cross3(axes[4]);
      axes[14] = axes[2].cross3(axes[5]);

      bool gape = false;
      for(auto &ax : axes){
        bool gap = hasGap(ax,cb1,cb2);
        if(gap){
          gape = true;
          break;
        }
      }
      if(!gape){
        collision = true;
      }
    }
    return collision;
  }

  void partial_ik(float j1e,float j234e,float j5e,float Px,float Py,float Pz,std::vector<Solution> &sols){
    
    float j3e = j3(j1e,j5e,j234e,Px,Py,Pz);
    auto j2e = j2(j3e,j1e,j5e,j234e,Px,Py,Pz);
    float j4_e1 = j234e - j3e - j2e.first;
    float j4_e2 = j234e + j3e - j2e.second;
    //sols.push_back({.j1=j1e,.j2=j2e.first,.j3=j3e,.j4=j4_e1,.j5=j5e});
    //sols.push_back({.j1=j1e,.j2=j2e.second,.j3=-j3e,.j4=j4_e2,.j5=j5e});
    sols.push_back({.j1=getOptimalAngle(j1e),.j2=getOptimalAngle(j2e.first),.j3=getOptimalAngle(j3e),.j4=getOptimalAngle(j4_e1),.j5=getOptimalAngle(j5e)});
    sols.push_back({.j1=getOptimalAngle(j1e),.j2=getOptimalAngle(j2e.second),.j3=getOptimalAngle(-j3e),.j4=getOptimalAngle(j4_e2),.j5=getOptimalAngle(j5e)});
  }

  void full_ik(Matrix4f &pose_matrix){
    float txx = pose_matrix(0,0);
    float txy = pose_matrix(0,1);
    float txz = pose_matrix(0,2);
    float tyx = pose_matrix(1,0);
    float tyy = pose_matrix(1,1);
    float tyz = pose_matrix(1,2);
    float tzx = pose_matrix(2,0);
    float tzy = pose_matrix(2,1);
    float tzz = pose_matrix(2,2);
    float Px = pose_matrix(0,3);
    float Py = pose_matrix(1,3);
    float Pz = pose_matrix(2,3);

    std::vector<Solution> sols;

    float j1e = j1(tyy,txy);
    float j5e = j5(tzz,tzx);
    float j234e = j234(tzx,tzy,j5e);

    partial_ik(j1e,j234e,j5e,Px,Py,Pz,sols);
  }

  void position_ik(float Px,float Py,float Pz, std::vector<Solution> &sols){
    std::vector<Solution> all_sols;
    for(int j1d=-90;j1d<=90;j1d+=10){
        for(int j234d=-179;j234d<=179;j234d+=10){
            float j1e = j1d*(M_PI/180);
            float j234e = j234d*(M_PI/180);
            float j5e = j5_new(j1e,Px,Py);
            partial_ik(j1e,j234e,j5e,Px,Py,Pz,all_sols);
        }
    }
    bool sol_found = false;
    for(auto &sol : all_sols){
        bool isValid =  checkPositionSolution(sol.j1,sol.j2,sol.j3,sol.j4,sol.j5,Px,Py,Pz);
        sol_found = sol_found or isValid;
        if(isValid){
        //std::cout<<sol.j1*(180.0/M_PI)<<" "<<sol.j2*(180.0/M_PI)<<" "<<sol.j3*(180.0/M_PI)<<" "<<sol.j4*(180.0/M_PI)<<" "<<sol.j5*(180.0/M_PI)<<std::endl;
          sols.push_back(sol);
        }
    }
  }

  Solution findBestSolution(std::vector<Solution> &sols){
    std::sort(sols.begin(),sols.end(),compareFn);
    std::cout<<"New solution"<<std::endl;
    for(auto &sol : sols){
        std::cout<<sol.j1*(180.0/M_PI)<<" "<<sol.j2*(180.0/M_PI)<<" "<<sol.j3*(180.0/M_PI)<<" "<<sol.j4*(180.0/M_PI)<<" "<<sol.j5*(180.0/M_PI)<<std::endl;
    }
    return sols[0];
  }

  static bool compareFn(Solution &a,Solution &b){
    //0.02 is 1 degree
    // Can be upgraded to include dot product to the desired output direction.
    //The smaller the dot product the better
    //We can have a map of desired output directions based on XYZ.
    //Although we are naturally getting human like shots which is super cool
    int ans = compareHelper(a.j2,b.j2,ConfigStore::j2_start,
    compareHelper(a.j3,b.j3,ConfigStore::j3_start,
    compareHelper(a.j1,b.j1,ConfigStore::j1_start,
    compareHelper(a.j4,b.j4,ConfigStore::j4_start,
    compareHelper(a.j5,b.j5,ConfigStore::j5_start,-1)))));
    if(ans != -1 ){
      return ans;
    }
    //If above returns -1 that means that all the angles are very close to each other in absolute terms
    // Then we need to bias them towards positive angles
    return compareHelper2(a.j1,b.j1,
    compareHelper2(a.j2,b.j2,
    compareHelper2(a.j3,b.j3,
    compareHelper2(a.j4,b.j4,
    compareHelper2(a.j5,b.j5,1)))));   
  }
  
  // All inputs in radian.
  static int compareHelper(float a1,float a2,float startAngle,int fallForward){
    float movement1 = abs(a1 - startAngle);
    float movement2 = abs(a2 - startAngle);
    // Movement is basically the same, defer to other angles.
    if(abs(movement1 - movement2) < 0.350){
      return fallForward;
    }
    else{
      // Movement is different. Bias towards minimal movement.
      return movement1 < movement2;
    }
  }

  static int compareHelper2(float a1,float a2,int fallForward){
    if(abs(a1 - a2) < 0.350){
      return fallForward;
    }
    return a1 > a2;
  }

  float searchYZero(const EstimatedParams &params){
    float t1 = 0;
    float t2 = 10;
    while(abs(t1-t2) > 0.001){
      float te = (t1+t2)/2.0;
      double xe,ye,ze;
      TrajectoryStore::getPredictedPointAtTime2(params,te,xe,ye,ze);
      if(abs(ye) < 0.01){
        return te;
      }
      else if(ye > 0){
        t1 = te;
      }
      else{
        t2 = te;
      }
    }
    return t1;
  }


  void trajectory_ik(float x0,float y0,float z0,float vx,float vy,float vz,std::vector<Solution> &sols){
      const EstimatedParams params = {.vx = vx,.vy = vy,.vz = vz,.x0 = x0,.y0 = y0,.z0 = z0,.g = 9.8,.error = 0,};
    float te = searchYZero(params);
    for(float t = te - 1; t <= te + 1;t += 0.01){
      double xe,ye,ze;
      TrajectoryStore::getPredictedPointAtTime2(params,t,xe,ye,ze);
      //cout<<xe<<" "<<ye<<" "<<ze<<endl;
      position_ik(xe,ye,ze,sols);
    }
    findBestSolution(sols);
  }

  

  float getOptimalAngle(float angle)
  {
    float a1 = angle;
    float a1d = abs(angle);
    float a2 = angle + 2 * M_PI;
    float a2d = abs(a2);
    float a3 = angle - 2 * M_PI;
    float a3d = abs(a3);
    if (a1d <= a2d)
    {
      return a1d <= a3d ? a1 : a3;
    }
    else
    {
      return a2d <= a3d ? a2 : a3;
    }
  }

  
  void initializeCubes(){
    Cuboid &cb1 = cuboids[0];
    Cuboid &cb2 = cuboids[1];
    Cuboid &cb3 = cuboids[2];
    Cuboid &cb4 = cuboids[3];
    Cuboid &cb5 = cuboids[4];

    // Base cube
    cb1(0,0) = -0.15;
    cb1(1,0) = -0.15;
    cb1(2,0) = 0.66;
    cb1(3,0) = 1;

    cb1(0,1) = 0.15;
    cb1(1,1) = -0.15;
    cb1(2,1) = 0.66;
    cb1(3,1) = 1;

    cb1(0,2) = 0.15;
    cb1(1,2) = -0.15;
    cb1(2,2) = 0;
    cb1(3,2) = 1;

    cb1(0,3) = -0.15;
    cb1(1,3) = -0.15;
    cb1(2,3) = 0;
    cb1(3,3) = 1;

    cb1(0,4) = -0.15;
    cb1(1,4) = 0.15;
    cb1(2,4) = 0.66;
    cb1(3,4) = 1;

    cb1(0,5) = 0.15;
    cb1(1,5) = 0.15;
    cb1(2,5) = 0.66;
    cb1(3,5) = 1;

    cb1(0,6) = 0.15;
    cb1(1,6) = 0.15;
    cb1(2,6) = 0;
    cb1(3,6) = 1;

    cb1(0,7) = -0.15;
    cb1(1,7) = 0.15;
    cb1(2,7) = 0;
    cb1(3,7) = 1;

    // Arm 1
    cb2(0,0) = -0.05;
    cb2(1,0) = -0.15;
    cb2(2,0) = 0.3;
    cb2(3,0) = 1;

    cb2(0,1) = 0.05;
    cb2(1,1) = -0.15;
    cb2(2,1) = 0.3;
    cb2(3,1) = 1;

    cb2(0,2) = 0.05;
    cb2(1,2) = -0.15;
    cb2(2,2) = 0;
    cb2(3,2) = 1;

    cb2(0,3) = -0.05;
    cb2(1,3) = -0.15;
    cb2(2,3) = 0;
    cb2(3,3) = 1;

    cb2(0,4) = -0.05;
    cb2(1,4) = 0.15;
    cb2(2,4) = 0.3;
    cb2(3,4) = 1;

    cb2(0,5) = 0.05;
    cb2(1,5) = 0.15;
    cb2(2,5) = 0.3;
    cb2(3,5) = 1;

    cb2(0,6) = 0.05;
    cb2(1,6) = 0.15;
    cb2(2,6) = 0;
    cb2(3,6) = 1;

    cb2(0,7) = -0.05;
    cb2(1,7) = 0.15;
    cb2(2,7) = 0;
    cb2(3,7) = 1;

    // Arm 3
    cb3(0,0) = -0.05;
    cb3(1,0) = -0.15;
    cb3(2,0) = 0.3;
    cb3(3,0) = 1;

    cb3(0,1) = 0.05;
    cb3(1,1) = -0.15;
    cb3(2,1) = 0.3;
    cb3(3,1) = 1;

    cb3(0,2) = 0.05;
    cb3(1,2) = -0.15;
    cb3(2,2) = 0;
    cb3(3,2) = 1;

    cb3(0,3) = -0.05;
    cb3(1,3) = -0.15;
    cb3(2,3) = 0;
    cb3(3,3) = 1;

    cb3(0,4) = -0.05;
    cb3(1,4) = 0.15;
    cb3(2,4) = 0.3;
    cb3(3,4) = 1;

    cb3(0,5) = 0.05;
    cb3(1,5) = 0.15;
    cb3(2,5) = 0.3;
    cb3(3,5) = 1;

    cb3(0,6) = 0.05;
    cb3(1,6) = 0.15;
    cb3(2,6) = 0;
    cb3(3,6) = 1;

    cb3(0,7) = -0.05;
    cb3(1,7) = 0.15;
    cb3(2,7) = 0;
    cb3(3,7) = 1;

    // Arm 4
    cb4(0,0) = -0.05;
    cb4(1,0) = -0.15;
    cb4(2,0) = 0.1;
    cb4(3,0) = 1;

    cb4(0,1) = 0.05;
    cb4(1,1) = -0.15;
    cb4(2,1) = 0.1;
    cb4(3,1) = 1;

    cb4(0,2) = 0.05;
    cb4(1,2) = -0.15;
    cb4(2,2) = 0;
    cb4(3,2) = 1;

    cb4(0,3) = -0.05;
    cb4(1,3) = -0.15;
    cb4(2,3) = 0;
    cb4(3,3) = 1;

    cb4(0,4) = -0.05;
    cb4(1,4) = 0.15;
    cb4(2,4) = 0.1;
    cb4(3,4) = 1;

    cb4(0,5) = 0.05;
    cb4(1,5) = 0.15;
    cb4(2,5) = 0.1;
    cb4(3,5) = 1;

    cb4(0,6) = 0.05;
    cb4(1,6) = 0.15;
    cb4(2,6) = 0;
    cb4(3,6) = 1;

    cb4(0,7) = -0.05;
    cb4(1,7) = 0.15;
    cb4(2,7) = 0;
    cb4(3,7) = 1;

    // Arm 5
    // This cube is interesting because it encodes the entire 360 degree rotation of the bat.
    // That makes collision detection so much faster, and I can rev up the bat as fast as I want
    // To hit the ball with full speed. So many similar cool implementation details all over this codebase.
    cb5(0,0) = -0.9;
    cb5(1,0) = -0.9;
    cb5(2,0) = 0.1;
    cb5(3,0) = 1;

    cb5(0,1) = 0.9;
    cb5(1,1) = -0.9;
    cb5(2,1) = 0.1;
    cb5(3,1) = 1;

    cb5(0,2) = 0.9;
    cb5(1,2) = -0.9;
    cb5(2,2) = 0;
    cb5(3,2) = 1;

    cb5(0,3) = -0.9;
    cb5(1,3) = -0.9;
    cb5(2,3) = 0;
    cb5(3,3) = 1;

    cb5(0,4) = -0.9;
    cb5(1,4) = 0.9;
    cb5(2,4) = 0.1;
    cb5(3,4) = 1;

    cb5(0,5) = 0.9;
    cb5(1,5) = 0.9;
    cb5(2,5) = 0.1;
    cb5(3,5) = 1;

    cb5(0,6) = 0.9;
    cb5(1,6) = 0.9;
    cb5(2,6) = 0;
    cb5(3,6) = 1;

    cb5(0,7) = -0.9;
    cb5(1,7) = 0.9;
    cb5(2,7) = 0;
    cb5(3,7) = 1;
  }

  IKSolver()
  {
    fkMatrices = new FKMatrices();
    FK = Eigen::MatrixXf::Identity(4, 4);
    T1 = Eigen::MatrixXf::Identity(4, 4);
    T2 = Eigen::MatrixXf::Identity(4, 4);
    T3 = Eigen::MatrixXf::Identity(4, 4);
    T4 = Eigen::MatrixXf::Identity(4, 4);
    T5 = Eigen::MatrixXf::Identity(4, 4);
    cuboids = std::vector<Cuboid>(5);

    initializeCubes();
  }
};