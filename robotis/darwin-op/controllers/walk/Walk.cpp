// Copyright 1996-2022 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
When running this controller in the real robot
do not interface via SSh client (i,e, PuTTY)
The result will be on a segmentation fault error.

Instead interface with the robot via remote desktop
(i.e. VNC). Open a terminal window, compile controller
and run.
*/
#include "Walk.hpp"

#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Lidar.hpp>
#include <webots/DistanceSensor.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <algorithm>

using namespace webots;
using namespace managers;
using namespace std;

#define M_PI 3.14159265358979323846

#define NOT_REVOLVE float('inf')

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */,
  "ArmLowerL" /*ID6 */, "PelvYR" /*ID7 */,    "PelvYL" /*ID8 */,    "PelvR" /*ID9 */,     "PelvL" /*ID10*/,
  "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR" /*ID15*/,
  "AnkleL" /*ID16*/,    "FootR" /*ID17*/,     "FootL" /*ID18*/,     "Neck" /*ID19*/,      "Head" /*ID20*/
};

bool isAutoMove() { // TODO: 自由移动
  return false;
}

vector<int> getShowOrder(int start, int end) {
  vector<int> output_ordert = {};
  if (start == end) {
    output_ordert.push_back(start);
    return output_ordert;
  }
  if (start == 3 && end == 6) {
    output_ordert = {3, 2, 6};
    return output_ordert;
  }
  if (start == 6 && end == 3) {
    output_ordert = {6, 2, 3};
    return output_ordert;
  }
  vector<int> current_order1 = {1, 0, 2, 3, 4, 5, 6};
  vector<int> current_order2 = {1, 0, 2, 6, 5, 4, 3};
  vector<int> current_order3 = {6, 5, 4, 3, 2, 0, 1};
  vector<int> current_order4 = {3, 4, 5, 6, 2, 0, 1};
  auto f = [](int start, int end, vector<int> current_order) {
    vector<int> output_order = {};
    bool findStart = false;
    for (int i: current_order) {
      if (i == start) {
        output_order.push_back(i);
        findStart = true;
      }
      else if (findStart && i != end) {
        output_order.push_back(i);
      }
      else if (i == end && findStart) {
        output_order.push_back(i);
        break;
      }
      else if (i == end && !findStart) {
        output_order = {};
        break;
      }

    }
    return output_order;
  };
  vector<vector<int>> nonEmptyVectors;
  if (!f(start, end, current_order1).empty()) {nonEmptyVectors.push_back(f(start, end, current_order1));}
  if (!f(start, end, current_order2).empty()) {nonEmptyVectors.push_back(f(start, end, current_order2));}
  if (!f(start, end, current_order3).empty()) {nonEmptyVectors.push_back(f(start, end, current_order3));}
  if (!f(start, end, current_order4).empty()) {nonEmptyVectors.push_back(f(start, end, current_order4));}
  if (nonEmptyVectors.empty()) {
    return {};
  }

  // cout << "nonEmptyVectors" << nonEmptyVectors.size() << endl;
  // for (auto i: nonEmptyVectors) {
  //   for (auto j: i) {
  //     cout << j << " ";
  //   }
  //   cout << endl;
  // }

  int min_vec = 0;
  for (int i = 0; i < nonEmptyVectors.size(); ++i) {
    if (nonEmptyVectors[i].size() < nonEmptyVectors[min_vec].size()) {
      min_vec = i;
    }
  }
  return nonEmptyVectors[min_vec];
}


Walk::Walk() : Robot() {
  mTimeStep = getBasicTimeStep();

  getLED("HeadLed")->set(0xFF0000);
  getLED("EyeLed")->set(0x00FF00);
  mAccelerometer = getAccelerometer("Accelerometer");
  mGPS = getGPS("gps");
  mIMU = getInertialUnit("imu");
  mAccelerometer->enable(mTimeStep);
  mLidar = getLidar("Sick LMS 291");
  mDistanceSensors[0] = getDistanceSensor("front1");
  mDistanceSensors[1] = getDistanceSensor("front2");
  mDistanceSensors[2] = getDistanceSensor("left1");
  mDistanceSensors[3] = getDistanceSensor("left2");
  mDistanceSensors[4] = getDistanceSensor("right1");
  mDistanceSensors[5] = getDistanceSensor("right2");
  getGyro("Gyro")->enable(mTimeStep);

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
  }

  mKeyboard = getKeyboard();
  mKeyboard->enable(mTimeStep);
  mGPS->enable(mTimeStep);
  mIMU->enable(mTimeStep);
  mLidar->enable(mTimeStep);
  mLidar->enablePointCloud();

  for (int i = 0; i < 6; i++)
    mDistanceSensors[i]->enable(mTimeStep);

  mMotionManager = new RobotisOp2MotionManager(this);
  mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
}

/*          x
            |
            |
            |
  y<-----------------   逆时针 yaw角增大，初始为0
            |
            |  
*/

void Walk::GetNowPosition(){
  const double *position = mGPS->getValues();
  now_position.x = position[0];
  now_position.y = position[1];
  now_yaw = mIMU->getRollPitchYaw()[2];
  // cout<<"yaw: "<<now_yaw<<endl;
  // cout<<"x: "<<now_position.x<<" y: "<<now_position.y<<endl;
}

void Walk::Go2Point(Point target_point){
  // get the current position of the robot
  double x = now_position.x;
  double y = now_position.y;
  double theta = now_yaw;
  double x_target = target_point.x;
  double y_target = target_point.y;
  double theta_target = atan2(y_target - y, x_target - x);
  double distance = sqrt(pow(x_target - x, 2) + pow(y_target - y, 2));
  double angle = theta_target - theta;
  if (angle > M_PI)
    angle -= 2 * M_PI;
  if (angle < -M_PI)
    angle += 2 * M_PI;

  // cout<<"distance: "<<distance<<" angle: "<<angle<<endl;
  if (distance > 0.1) {
    mGaitManager->setXAmplitude(1.0);
    mGaitManager->setAAmplitude(angle);
  } else {
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
  }
}

float thre = 4.5;
void Walk::Go2PointBug0(Point target_point){
  // get the current position of the robot
  double x = now_position.x;
  double y = now_position.y;
  double theta = now_yaw;
  double x_target = target_point.x;
  double y_target = target_point.y;
  double theta_target = atan2(y_target - y, x_target - x);
  double distance = sqrt(pow(x_target - x, 2) + pow(y_target - y, 2));
  double angle = theta_target - theta;
  if (angle > M_PI)
    angle -= 2 * M_PI;
  if (angle < -M_PI)
    angle += 2 * M_PI;




  // 基于激光雷达的深度信息，判断是否有障碍物
  // 有障碍物，就绕行
  // 没有障碍物，直接走向目标点

  // 1. 基于激光雷达的深度信息，发现深度突变的地方，就是障碍物的边缘
  int i=-1;

  for(int j=45; j<135; j++){

    if(lidar_depths[j+1] > 2.0){
      continue;
    }
    if(lidar_depths[j] - lidar_depths[j+1] > 0.8){
      // 有障碍物
      i = j+1; // 记录障碍物左侧边缘的角度
      break;

  }


  }

  if(fabs(angle) > M_PI/6 && lidar_depths[i] > 0.8){
    Go2Point(target_point);
    return;
  }

  

  if (i != -1)
  {
    // cout << "障碍物角度: " << i << endl;
    // cout<<"障碍物深度: "<<lidar_depths[i]<<endl;
    // cout<<"yaw: "<<now_yaw<<endl;
    
    // cout<<"x: "<<x<<" y: "<<y<<endl;
    // 2. 计算障碍物的边缘点的坐标
    float x_obstacle = x + lidar_depths[i] * sin(i * M_PI / 180 + now_yaw);
    float y_obstacle = y + lidar_depths[i] * cos(i * M_PI / 180 + now_yaw);


    // cout<<"目标角度:"<<theta_target<<endl;

    // 3. 检查障碍物边缘点是否在目标点的前方
    // 如果在目标点的前方，就绕行
    // 如果在目标点的后方，就直接走向目标点
    int obstacled = 0;
    int ratio1 = 1;
    float temp_yaw = now_yaw + M_PI/2;
    if(temp_yaw > M_PI){
      temp_yaw -= 2 * M_PI;
    }
    if(temp_yaw < -M_PI){
      temp_yaw += 2 * M_PI;
    }

    // cout<<"temp_yaw: "<<temp_yaw<<endl;

    if(temp_yaw > M_PI/3 && temp_yaw < 2 * M_PI/3){
        if(x_obstacle < x_target){
          // cout<<"障碍物在目标点的前方"<<endl;
          obstacled = 1;
          ratio1 = 1;
      }
    }
    else if(temp_yaw > -2 * M_PI/3 && temp_yaw < -M_PI/3){
        if(x_obstacle > x_target){
          // cout<<"障碍物在目标点的前方"<<endl;
          obstacled = 1;
          ratio1 = -1;
      }
    }
    else if(temp_yaw > -M_PI/3 && temp_yaw < M_PI/3){
        if(y_obstacle > y_target){
          // cout<<"障碍物在目标点的前方"<<endl;
          obstacled = 1;
          ratio1 = -1;
      }
    }
    else{
        if(y_obstacle < y_target){
          // cout<<"障碍物在目标点的前方"<<endl;
          obstacled = 1;
          ratio1 = 1;
      }
    }




    // if (x_obstacle * sin(theta_target + 1.5708) - x_target * sin(theta_target + 1.5708) + 
    //       y_target * cos(theta_target + 1.5708) - y_obstacle * cos(theta_target + 1.5708) < 0)
    if(obstacled)
    {
      // 障碍物在目标点的前方
      // 绕行
      // cout << "绕行" << endl;
      // cout<<"ratio1:"<<ratio1<<endl;
      // cout<<"x_obstacle: "<<x_obstacle<<" y_obstacle: "<<y_obstacle<<endl;
      Go2Point({x_obstacle - thre*ratio1, y_obstacle + thre*ratio1});
    }
    else
    {
      // 障碍物在目标点的后方
      // 直接走向目标点
      Go2Point(target_point);
    }

  }
  else
  {
    // 没有障碍物
    // 直接走向目标点
    Go2Point(target_point);
  }

}

void Walk::RevolveYaw(fp32 target_yaw)
{
  if (target_yaw == NOT_REVOLVE) {
    return;
  }
  double angle = target_yaw - now_yaw;
  if (angle > M_PI)
    angle -= 2 * M_PI;
  if (angle < -M_PI)
    angle += 2 * M_PI;
  mGaitManager->setXAmplitude(0.0);
  mGaitManager->setAAmplitude(0.5*angle);
}

Walk::~Walk() {
}

void Walk::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}


void Walk::RaiseArmToShow(bool &isWalking){
    if (isWalking) {
      mGaitManager->stop();
      mMotors[2]->setPosition(-0.68);
      mMotors[4]->setPosition(-1.65);
      mMotors[0]->setPosition(2.3);
      wait(200);
    } else {
      mGaitManager->start();
      isWalking = true;
      wait(200);
    }
}



void Walk::GetDistanceSensorsValues()
{
  for (int i = 0; i < 6; i++)
  {
    distance_sensors_values[i] = mDistanceSensors[i]->getValue();
  }

  // cout<<"Distance Sensor 0: "<<distance_sensors_values[0]<<endl;
  // cout<<"Distance Sensor 1: "<<distance_sensors_values[1]<<endl;

}


void Walk::GetLidarData(){
  lidar_depths = mLidar->getRangeImage();

  // cout<<"Lidar Depth 90 degree: "<<lidar_depths[90]<<endl;
  // cout<<"Lidar getNumberOfLayers: "<<mLidar->getNumberOfLayers()<<endl;  // 1
  // cout<<"Lidar Depth Number: "<<sizeof(lidar_depths)/sizeof(lidar_depths[0])<<endl;
}



// function containing the main feedback loop
void Walk::run() {
  cout << "-------Walk example of ROBOTIS OP2-------" << endl;
  cout << "This example illustrates Gait Manager" << endl;
  cout << "Press the space bar to start/stop walking" << endl;
  cout << "Use the arrow keys to move the robot while walking" << endl;
  // First step to update sensors values
  myStep();

  // play the hello motion
  mMotionManager->playPage(9);  // init position
  wait(200);

  // main loop
  bool isWalking = false;

  while (true) {
    checkIfFallen();
    GetNowPosition();

    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);

    // get keyboard key
    int key = 0;
    while ((key = mKeyboard->getKey()) >= 0) {
      switch (key) {
        case ' ':  // Space bar
          if (isWalking) {
            mGaitManager->stop();
            isWalking = false;
            wait(200);
          } else {
            mGaitManager->start();
            isWalking = true;
            wait(200);
          }
          break;
        case Keyboard::UP:
          mGaitManager->setXAmplitude(1.0);
          break;
        case Keyboard::DOWN:
          mGaitManager->setXAmplitude(-1.0);
          break;
        case Keyboard::RIGHT:
          mGaitManager->setAAmplitude(-0.5);
          break;
        case Keyboard::LEFT:
          mGaitManager->setAAmplitude(0.5);
          break;
        case 'Q':
          RaiseArmToShow(isWalking); // isWalking = True -> False
          break;
        case 'K':
          Go2Point({3.52, 2.2}); // isWalking = True
          break;
      }
    }

    mGaitManager->step(mTimeStep);

    // step
    myStep();
  }
}

void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;

  // count how many steps the accelerometer
  // says that the robot is down
  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;

  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;

  // the robot face is down
  if (fup > acc_step) {
    mMotionManager->playPage(10);  // f_up
    mMotionManager->playPage(9);   // init position
    fup = 0;
  }
  // the back face is down
  else if (fdown > acc_step) {
    mMotionManager->playPage(11);  // b_up
    mMotionManager->playPage(9);   // init position
    fdown = 0;
  }
}

//----------Ke's code begin----------

fp32 get_distance(Point current, Point target) { //未声明
  return pow(pow(current.x-target.x, 2) + pow(current.y-target.y, 2), 0.5);
}

PathPlanning::PathPlanning() {
  PathPlanning::robotStatu = START;
  PathPlanning::current_step = 0;
  PathPlanning::show_order = {};
  key_points = {
    {{3.2, 3.3}, NOT_REVOLVE},
    {{6.2, 3.3}, 0.0f},
    {{3.5, 0.3}, -M_PI/2.0f},
    {{5.8, 0.3}, 0.0f},
    {{5.8, -3.2}, 0.0f},
    {{1.5, -3.2}, M_PI},
    {{1.5, 0.3}, M_PI}
  };
  PathPlanning::controller = new Walk();
}

PathPlanning::PathPlanning(std::vector<int> show_order) {
  PathPlanning::robotStatu = START;
  PathPlanning::current_step = 0;
  PathPlanning::show_order = show_order;
  key_points = {
    {{3.5, 3.3}, NOT_REVOLVE},
    {{6.2, 3.3}, 0.0f},
    {{3.5, 0.3}, -M_PI/2.0f},
    {{5.8, 0.3}, 0.0f},
    {{5.8, -3.2}, 0.0f},
    {{1.5, -3.2}, M_PI},
    {{1.5, 0.3}, M_PI}
  };
  PathPlanning::controller = new Walk();
}

PathPlanning::~PathPlanning() {
  delete PathPlanning::controller;
}

void PathPlanning::showInOrder() {
  cout << "Press the space bar to start/stop walking" << endl;
  cout << "Use the arrow keys to move the robot while walking" << endl;

  // First step to update sensors values
  controller->myStep();
  // play the hello motion
  controller->mMotionManager->playPage(9);  // init position
  controller->wait(200);
  // main loop
  bool isWalking = true;

  bool isWorking = false;
  int current_key = -1;
  int last_current_key = -1;
  int current_p = 0;
  while (true) {
    controller->checkIfFallen();
    controller->GetNowPosition();
    controller->GetDistanceSensorsValues();
    controller->GetLidarData();
    controller->mGaitManager->setXAmplitude(0.0);
    controller->mGaitManager->setAAmplitude(0.0);

    // get keyboard key
    int key = 0;
    while ((key = controller->mKeyboard->getKey()) >= 0) {
      switch (key) {
        case ' ':  // Space bar
          // cout << "keyboard  " << endl;
          if (isWalking) {
            controller->mGaitManager->stop();
            isWalking = false;
            controller->wait(200);
          } else {
            controller->mGaitManager->start();
            isWalking = true;
            controller->wait(200);
          }
          break;
        case 'G':
          isWorking = true;
          break;
        case 'S':
          isWorking = false;
          break;
        case '0':
          current_key = 0;
          break;
        case '1':
          current_key = 1;
          break;
        case '2':
          current_key = 2;
          break;
        case '3':
          current_key = 3;
          break;
        case '4':
          current_key = 4;
          break;
        case '5':
          current_key = 5;
          break;
        case '6':
          current_key = 6;
          break;
        // case 'T':
        //   current_key = 10;
          break;
        default:
          break;
      }
    }
    if (controller->mKeyboard->getKey() == -1)  //没有键盘键入
    {
      // cout<<"current_key: "<<current_key<<endl;
      // cout<<"current_p: "<<current_p<<endl;
      if (isWorking) 
      {
        if (current_key != last_current_key || current_key == -1) 
        {
          int current_point = 0;
          float current_distance = 1000.0f;
          for (int i = 0; i <= 6; ++i) {
            if (get_distance(controller->now_position, key_points[i].p) < current_distance) {
              current_point = i;
              current_p = i;
              current_distance = get_distance(controller->now_position, key_points[i].p);
            }
          }
          vector<int> show_order = getShowOrder(current_point, current_key);
          // controller->wait(200);

          // for (int i: show_order) {
          //   cout << i << " ";
          // }
          // cout << endl;

          if (!show_order.empty()) 
          {
            PathPlanning::show_order = show_order;
            PathPlanning::robotStatu = START;
            current_step = 0;
          }
          last_current_key = current_key;
          current_key = -1;
        }
        // cout << "keyboard g" << endl;
        switch (PathPlanning::robotStatu)
        {
        case RobotStatu_e::START:
          if (current_step < int(show_order.size())) {
            PathPlanning::robotStatu = RobotStatu_e::RUNNING;
          }
          else {
            PathPlanning::robotStatu = RobotStatu_e::OFF;
          }
          break;
        case RobotStatu_e::RUNNING:
          // cout << "RUNNING" << endl;
          controller->GetNowPosition();
          if (get_distance(controller->now_position, key_points[show_order[current_step]].p) < 0.2) {
            PathPlanning::robotStatu = RobotStatu_e::REVOLVE;
          }
          else {
            controller->Go2PointBug0(key_points[show_order[current_step]].p);
          }
          if (isAutoMove()) {
            PathPlanning::robotStatu = RobotStatu_e::AUTO_MOVE;
          }
          break;

        case RobotStatu_e::AUTO_MOVE:
          if (!isAutoMove()) {
            PathPlanning::robotStatu = RobotStatu_e::RUNNING;
          }
          break;

        case RobotStatu_e::REVOLVE:
          if ((fabs(controller->now_yaw - key_points[show_order[current_step]].yaw) < 0.1 || fabs(fabs(controller->now_yaw - key_points[show_order[current_step]].yaw) - 2*M_PI)  < 0.1 ) || key_points[show_order[current_step]].yaw == NOT_REVOLVE || current_step != show_order.size() - 1) {
            if ((current_step == show_order.size() - 1) && show_order[current_step] != 0) {
              PathPlanning::robotStatu = RobotStatu_e::SHOW;
            }
            
            else {
              if (show_order[current_step] != 0) {
                current_step++;
                PathPlanning::robotStatu = RobotStatu_e::START;
              }
              else {
                current_step++;
                PathPlanning::robotStatu = RobotStatu_e::START;
              }       
            }

          }
          else {
            controller->RevolveYaw(key_points[show_order[current_step]].yaw);
          }
          break;
        case RobotStatu_e::SHOW:
            // cout << "SHOW" << endl;
            controller->RaiseArmToShow(isWalking);
            current_step++;
            PathPlanning::robotStatu = RobotStatu_e::START;
            controller->wait(1000);
            controller->mGaitManager->start();
            isWalking = true;
            controller->wait(200);
          break;
        case RobotStatu_e::OFF:
          break;
        default:
          break;
        }
      }
    }
    controller->mGaitManager->step(controller->mTimeStep);

    // step
    controller->myStep();
    
    if (isWorking) {
      if (current_key != -1) {
        if ((current_step >= show_order.size() - 1) && PathPlanning::robotStatu == RobotStatu_e::OFF) {
          cout << "已到达" << show_order[show_order.size()-1] << "号展品"<< endl;
        }
        else {
        cout << "出发坐标: (" << key_points[current_p].p.x << "," << key_points[current_p].p.y << ")" << 
        " 目标坐标: (" << key_points[current_key].p.x << "," << key_points[current_key].p.y << ")" << endl;
        }
        if (show_order[show_order.size()-1] != 0) {
          cout << "正在前往展品："<<show_order[show_order.size()-1] << "号展品" <<endl;
        }
        else {
          cout << "返回0号点位"<< endl;
        }
      }
      else if (current_key == -1) {
        cout << "请选择目标展品" << endl;
      }

    }
    else {
      cout << "请按G键开启导航" << endl;
    }




  }


}
//----------Ke's code end----------