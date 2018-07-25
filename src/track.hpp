/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef CFSD18_COGNITION_TRACK_HPP
#define CFSD18_COGNITION_TRACK_HPP

#include <opendlv-standard-message-set.hpp>
#include <cluon-complete.hpp>
#include <Eigen/Dense>
#include <map>
#include <chrono>
#include <thread>

class Track {
 public:
  Track(std::map<std::string, std::string> commandlineArguments, cluon::OD4Session &od4);
  Track(Track const &) = delete;
  Track &operator=(Track const &) = delete;
  virtual ~Track();
  void receiveCombinedMessage(std::map<int,opendlv::logic::perception::GroundSurfaceArea>, cluon::data::TimeStamp);
  void nextContainer(cluon::data::Envelope &);

 private:
  void setUp(std::map<std::string, std::string> commandlineArguments);
  void tearDown();

  void run(Eigen::MatrixXf localPath, cluon::data::TimeStamp sampleTime);
  bool slamParams();
  Eigen::VectorXf curveFit(Eigen::MatrixXf matrix);
  Eigen::RowVector2f traceBackToClosestPoint(Eigen::RowVector2f, Eigen::RowVector2f, Eigen::RowVector2f);
  Eigen::MatrixXf placeEquidistantPoints(Eigen::MatrixXf, bool, int, float);
  Eigen::MatrixXf orderCones(Eigen::MatrixXf localPath);
  std::tuple<float, float> driverModelSteering(Eigen::MatrixXf, float, float);
  float driverModelSharp(Eigen::MatrixXf, float);
  float driverModelVelocity(Eigen::MatrixXf, float, float);
  std::vector<float> curvatureTriCircle(Eigen::MatrixXf, int);
  std::vector<float> curvaturePolyFit(Eigen::MatrixXf);
  std::tuple<bool, Eigen::MatrixXf> pathMemory(Eigen::MatrixXf localPath);
  void curveDetectionNoSlam(Eigen::MatrixXf localPath);
  Eigen::MatrixXf pathProcessing(Eigen::MatrixXf localPath);

  /* commandlineArguments */
  cluon::OD4Session m_od4BB{219};
  cluon::OD4Session &m_od4;
  int m_senderStamp{221};
  // path
  double m_receiveTimeLimit{0.01};
  float m_distanceBetweenPoints{0.5f};
  bool m_traceBack{false};
  // steering
  bool m_moveOrigin{true};
  bool m_curveFitPath{true};
  bool m_ignoreOnePoint{true};
  bool m_calcDtaOnX{false};
  float m_previewTime{0.3f};
  float m_minPrevDist{1.0f};
  float m_steerRate{50.0f};
  float m_goodPathLimit{5.0f};
  float m_curveSteerAmpLim{10.0f};
  float m_prevReqRatio{0.0f};
  float m_curveDetectionAngle{0.3f};
  float m_curveExitAngleLim{0.1f};
  float m_previewTimeSlam{0.3f};
  float m_minPrevDistSlam{1.0f};
  float m_steerRateSlam{50.0f};
  //sharp
  bool m_sharp{false};
  int m_nSharp{10};
  float m_K1{0.17f};
  float m_Ky{0.5f};
  float m_C{m_K1};
  float m_c{1.0f};
  // velocity control
  float m_diffToBrakeVel{0.5f};
  float m_critDiff{0.1f};
  float m_critDiff2{0.1f};
  float m_accFreq{10};
  float m_axSpeedProfile{-1.0f};
  bool m_useAyReading{true};
  float m_velocityLimit{5.0f};
  float m_mu{0.9f};
  float m_axLimitPositive{5.0f};
  float m_axLimitNegative{-5.0f};
  float m_headingErrorDependency{0.7f};
  float m_curveDetectionAngleSlam{1.0f};
  int m_curveDetectionPoints{20};
  //....controller
  float m_aimVel{5.0f};
  float m_keepConstVel{-1.0f};
  float m_keepConstVelSlam{-1.0f};
  float m_aKp{0.1f};
  float m_aKd{0.0f};
  float m_aKi{0.0f};
  float m_bKp{0.1f};
  float m_bKd{0.0f};
  float m_bKi{0.0f};
  float m_sKp{0.1f};
  float m_sKd{0.0f};
  float m_sKi{0.0f};
  // curvature estimation
  bool m_polyFit{false};
  int m_step{5};
  int m_polyDeg{3};
  int m_pointsPerSegment{15};
  bool m_segmentizePolyfit{false};
  // vehicle specific
  float m_wheelAngleLimit{20.0f};
  float m_wheelBase{1.53f};
  float m_frontToCog{0.765f};

  /* Member variables */
  float const m_PI = 3.14159265f;
  float m_groundSpeed;
  std::mutex m_groundSpeedMutex;
  float m_lateralAcceleration;
  std::mutex m_lateralAccelerationMutex;
  std::chrono::time_point<std::chrono::system_clock> m_tick;
  std::chrono::time_point<std::chrono::system_clock> m_tock;
  std::chrono::time_point<std::chrono::system_clock> m_tickDt;
  std::chrono::time_point<std::chrono::system_clock> m_tockDt;
  std::chrono::time_point<std::chrono::system_clock> m_steerTickDt;
  std::chrono::time_point<std::chrono::system_clock> m_steerTockDt;
  bool m_newClock;
  bool m_brakingState;
  bool m_accelerationState;
  bool m_rollingState;
  float m_timeToCritVel;
  float m_accClock;
  float m_minRadius;
  float m_apexRadius;
  bool m_specCase;
  float m_ei;
  float m_ePrev;
  float m_fullTime;
  bool m_start;
  float m_prevHeadingRequest;
  bool m_slamActivated;
  bool m_paramsUpdated;
  bool m_inLeftCurve;
  bool m_inRightCurve;
  bool m_STOP;
  Eigen::MatrixXf m_savedPath;
  Eigen::MatrixXf m_globalPath;
  Eigen::Vector2f m_location;
  std::mutex m_locationMutex;
  float m_heading;
  bool m_noPath;
  bool m_onePoint;
  std::string folderName;
  std::mutex m_sendMutex;
};

#endif
