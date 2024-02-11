#include "ik_solver.h"
#include <random>
#include <ctime>
#include <iostream>
#include <chrono>
#include <sw/redis++/redis++.h>
#include <iomanip>

using namespace std;

float LO = -1;
float HI = 1;
float radius = 0.8;

int main()
{
  IKSolver solver;
  std::random_device rd1;    // Seed with a non-deterministic source
  std::mt19937 gen1(rd1());  // Mersenne Twister engine for better randomness
  std::random_device rd2;    // Seed with a non-deterministic source
  std::mt19937 gen2(rd2());  // Mersenne Twister engine for better randomness
  std::random_device rd3;    // Seed with a non-deterministic source
  std::mt19937 gen3(rd3());  // Mersenne Twister engine for better randomness

  std::uniform_real_distribution<> dist1(-1, 1);
  std::uniform_real_distribution<> dist2(-1, 1);
  std::uniform_real_distribution<> dist3(0.2, 1);

  vector<float> xs(100000, 0);
  vector<float> ys(100000, 0);
  vector<float> zs(100000, 0);

  for (int i = 0; i < 100000; i++)
  {
    float x = dist1(gen1);
    float y = dist2(gen2);
    float z = dist3(gen3);
    float r = sqrt(x * x + y * y + z * z);
    x = x * (radius / r);
    y = y * (radius / r);
    z = z * (radius / r);
    xs[i] = x;
    ys[i] = y;
    zs[i] = z;
  }

  auto start = std::chrono::high_resolution_clock::now();
  Matrix4f fk;
  FKMatrices::fk(0,M_PI_2,0,-M_PI_2,0,fk);
  cout<<fk(0,3)<<" "<<fk(1,3)<<" "<<fk(2,3)<<endl;
  for (int i = 0; i < 1; i++)
  {
    std::vector<Solution> sols;
    //solver.position_ik(1.1025 ,-0.00401968 ,0.81,0,sols);

    solver.trajectory_ik(0.8,7.21219,1.5522,-0.0,-10.2628,-1.66987,sols);
    /*if(sols.size()){
      long t = (1000.0*sols[0].t);
      auto nownow = chrono::system_clock::now();
      auto timeToContact = nownow + chrono::milliseconds(t);
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(timeToContact - nownow);
      std::cout<<timeToContact.time_since_epoch().count()<<endl;
      std::cout<<nownow.time_since_epoch().count()<<endl;
      std::cout<<t<<" "<<duration.count()<<std::endl;
    }*/
    cout<<"final: "<<sols.size()<<endl;
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  std::cout << "Time taken: " << duration.count() / 1 << " milliseconds" << std::endl;
}