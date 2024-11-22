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

// Description:   Example showing how to use the gait manager
//                and keyboard inputs

#ifndef WALK_HPP
#define WALK_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>

namespace managers {
  class RobotisOp2MotionManager;
  class RobotisOp2GaitManager;
}  // namespace managers

namespace webots {
  class Motor;
  class PositionSensor;
  class LED;
  class Camera;
  class Accelerometer;
  class Gyro;
  class Keyboard;
  class Speaker;
};  // namespace webots

typedef float fp32;

typedef struct Point
{
  float x;
  float y;
}Point;

typedef struct PointWithYaw
{
  Point p;
  fp32 yaw;
}PointWithYaw;

//----------Ke's code begin----------
/*
*************************************
exhibit1 ** exhibit3        exhibit4
         **
  #1     **   #3              #4
         **             **
                        **
  #0         #2 exhibit2**
                        **
                        **
         **             **
         **  #6               #5
         **
  robot  ** exhibit6        exhibit5
*/

// AUTO_MOVE 和 RUNNIG 两种状态相互转换

typedef enum {
  START=0, //已启动，但原地踏步
  RUNNING=1, //直线运动中
  SHOW=2, //展示展品中
  OFF=3, //取消启动
  REVOLVE=4, //旋转时
  AUTO_MOVE=5, //自动移动
}RobotStatu_e;




//----------Ke's code end----------

class Walk : public webots::Robot {
public:
  Walk();
  virtual ~Walk();
  void run();
  void checkIfFallen();
  void RaiseArmToShow(bool &isWalking);
  void Go2Point(Point target_point);
  void RevolveYaw(fp32 target_yaw);
  void GetNowPosition();
  void GetLidarData();

  void Go2PointBug0(Point target_point);

  void GetDistanceSensorsValues();
  int mTimeStep;

  void myStep();
  void wait(int ms);

  webots::Motor *mMotors[NMOTORS];
  webots::PositionSensor *mPositionSensors[NMOTORS];
  webots::Accelerometer *mAccelerometer;
  webots::GPS *mGPS;
  webots::InertialUnit *mIMU;
  webots::Keyboard *mKeyboard;
  webots::Lidar *mLidar;
  webots::DistanceSensor *mDistanceSensors[6];
  managers::RobotisOp2MotionManager *mMotionManager;
  managers::RobotisOp2GaitManager *mGaitManager;

  Point now_position;
  float now_yaw;
  const float *lidar_depths;
  float distance_sensors_values[6];
};

//----------Ke's code begin----------

class PathPlanning {
private:
  RobotStatu_e robotStatu;
  int current_step;
  Walk *controller;

  std::vector<PointWithYaw> key_points;

public:
  PathPlanning();
  PathPlanning(std::vector<int> show_order);
  ~PathPlanning();
  void showInOrder();
  std::vector<int> show_order;
};
//----------Ke's code end----------
#endif
