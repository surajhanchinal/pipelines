#include "ik_solver.h"
#include <random>
#include <ctime>
#include <iostream>
#include <chrono>
#include <sw/redis++/redis++.h>
#include <iostream>
#include <vector>
#include <iomanip>

using namespace std;
using namespace sw::redis;

float LO = -1;
float HI = 1;
float radius = 0.8;

int main()
{
  IKSolver solver;
  ConnectionOptions opts;
  opts.host = "127.0.0.1";
  opts.port = 6379;
  opts.socket_timeout = std::chrono::milliseconds(1);

  auto redis = Redis(opts);

  auto redis2 = Redis("tcp://127.0.0.1:6379");

  Subscriber sub = redis.subscriber();

  sub.subscribe("traj-channel");
  sub.on_message([&solver, &redis2](std::string channel, std::string msg) {
    istringstream iss(msg);
    vector<float> position;
    while (!iss.eof())
    {
      float value;
      iss >> value;
      position.push_back(value);
    }
    vector<Solution> sols;
    solver.trajectory_ik(position[3], position[4], position[5], position[0], position[1], position[2], sols);
    //Single solution mode
    /*if (sols.size())
    {
      auto& sol = sols[0];
      std::ostringstream oss;

      // Set the precision for the output stream
      oss << std::fixed << std::setprecision(0);
      oss << "MR" << " ";
      oss << (sol.j1-ConfigStore::j1_start)*(2000/M_PI) << " ";
      oss << (sol.j2 - ConfigStore::j2_start)*(1800/M_PI) << " ";
      oss << (sol.j3 - ConfigStore::j3_start)*(1800/M_PI) << " ";
      oss << (sol.j4 - ConfigStore::j4_start)*(1800/M_PI) << " ";
      oss << (sol.j5 - ConfigStore::j5_start)*(1800/M_PI) << " ";
      redis2.publish("solution", oss.str());
    }*/
    // Multiple solutions mode
    if(sols.size()){
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(0);
      for(int i=0;i<sols.size();i++){
        auto &sol = sols[i];
        oss << "MR" << " ";
        oss << (sol.j1-ConfigStore::j1_start)*(2000/M_PI) << " ";
        oss << (sol.j2 - ConfigStore::j2_start)*(1800/M_PI) << " ";
        oss << (sol.j3 - ConfigStore::j3_start)*(1800/M_PI) << " ";
        oss << (sol.j4 - ConfigStore::j4_start)*(1800/M_PI) << " ";
        oss << (sol.j5 - ConfigStore::j5_start)*(1800/M_PI) << " ";
        if(i != sols.size() - 1){
          oss << " | ";
        }
      }
      redis2.publish("solution", oss.str());
    }
  });

  while(true){
    try {
    sub.consume();
    }
    catch (const Error &err) {
        
    }
  }
}