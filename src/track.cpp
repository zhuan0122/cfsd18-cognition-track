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

#include <iostream>
#include <cmath>
#include "track.hpp"
#include <chrono>
#include <ctime>

Track::Track(std::map<std::string, std::string> commandlineArguments, cluon::OD4Session &od4) :
  m_od4(od4),
  m_groundSpeed{0.0f},
  m_groundSpeedMutex{},
  m_tickDt{},
  m_tockDt{},
  m_steerTickDt{},
  m_steerTockDt{},
  m_brakingState{false},
  m_accelerationState{false},
  m_rollingState{false},
  m_timeToCritVel{},
  m_accClock{0.0f},
  m_minRadius{1000000.0f},
  m_apexRadius{1000000.0f},
  m_specCase{false},
  m_ei{0.0f},
  m_ePrev{0.0f},
  m_fullTime{0.0f},
  m_prevHeadingRequest{0.0f},
  m_slamActivated{false},
  m_paramsUpdated{false},
  m_inLeftCurve{false},
  m_inRightCurve{false},
  m_STOP{false},
  m_noPath{},
  m_onePoint{},
  m_brakeInXSeconds{},
  m_critVel{},
  m_aimVelSet{false},
  m_prevState{false},
  m_tv{0.0f},
  m_case{0},
  m_preview{},
  m_minPreview{},
  m_groundSpeedReadingLeft{0.0f},
  m_groundSpeedReadingRight{0.0f},
  m_yawMutex{},
  m_yawRate{0.0f},
  m_sEPrev{0.0f},
  m_sEi{0.0f},
  m_aimClock{true},
  m_prevAngleToAimPoint{0.0f},
  m_prevAccelerationRequest{0.0f},
  folderName{},
  m_sendMutex()
{
 setUp(commandlineArguments);
 m_tickDt = std::chrono::system_clock::now();
 m_steerTickDt = std::chrono::system_clock::now();
 m_preview=m_previewTime;
 m_minPreview=m_minPrevDist;
}
Track::~Track()
{
}
void Track::setUp(std::map<std::string, std::string> commandlineArguments)
{
  m_speedId1=(commandlineArguments["speedId1"].size() != 0) ? (static_cast<uint32_t>(std::stoi(commandlineArguments["speedId1"]))) : (m_speedId1);
  m_speedId2=(commandlineArguments["speedId2"].size() != 0) ? (static_cast<uint32_t>(std::stoi(commandlineArguments["speedId2"]))) : (m_speedId2);
  m_senderStamp=(commandlineArguments["id"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["id"]))) : (m_senderStamp);
  // path
  m_distanceBetweenPoints=(commandlineArguments["distanceBetweenPoints"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["distanceBetweenPoints"]))) : (m_distanceBetweenPoints);
  // steering
  m_moveOrigin=(commandlineArguments["useMoveOrigin"].size() != 0) ? (std::stoi(commandlineArguments["useMoveOrigin"])==1) : (m_moveOrigin);
  m_orderPath=(commandlineArguments["useOrderPath"].size() != 0) ? (std::stoi(commandlineArguments["useOrderPath"])==1) : (m_orderPath);
  m_curveFitPath=(commandlineArguments["useCurveFitPath"].size() != 0) ? (std::stoi(commandlineArguments["useCurveFitPath"])==1) : (m_curveFitPath);
  m_curveFitDistance=(commandlineArguments["curveFitDistance"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["curveFitDistance"]))) : (m_curveFitDistance);
  m_ignoreOnePoint=(commandlineArguments["ignoreOnePointPaths"].size() != 0) ? (std::stoi(commandlineArguments["ignoreOnePointPaths"])==1) : (m_ignoreOnePoint);
  m_calcDtaOnX=(commandlineArguments["calcDtaOnX"].size() != 0) ? (std::stoi(commandlineArguments["calcDtaOnX"])==1) : (m_calcDtaOnX);
  m_previewTime=(commandlineArguments["previewTime"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["previewTime"]))) : (m_previewTime);
  m_minPrevDist=(commandlineArguments["minPrevDist"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["minPrevDist"]))) : (m_minPrevDist);
  m_constPrevDist=(commandlineArguments["constPrevDist"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["constPrevDist"]))) : (m_constPrevDist);
  m_steerRate=(commandlineArguments["steerRate"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["steerRate"]))) : (m_steerRate);
  m_curveSteerAmpLim=(commandlineArguments["curveSteerAmpLim"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["curveSteerAmpLim"]))) : (m_curveSteerAmpLim);
  m_prevReqRatio=(commandlineArguments["prevReqRatio"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["prevReqRatio"]))) : (m_prevReqRatio);
  m_curveDetectionAngle=(commandlineArguments["curveDetectionAngle"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["curveDetectionAngle"]))) : (m_curveDetectionAngle);
  m_curveExitAngleLim=(commandlineArguments["curveExitAngleLim"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["curveExitAngleLim"]))) : (m_curveExitAngleLim);
  m_previewTimeSlam=(commandlineArguments["previewTimeSlam"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["previewTimeSlam"]))) : (m_previewTimeSlam);
  m_minPrevDistSlam=(commandlineArguments["minPrevDistSlam"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["minPrevDistSlam"]))) : (m_minPrevDistSlam);
  m_steerRateSlam=(commandlineArguments["steerRateSlam"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["steerRateSlam"]))) : (m_steerRateSlam);
  m_prevReqRatio=(commandlineArguments["prevReqRatio"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["prevReqRatio"]))) : (m_prevReqRatio);
  m_aimRate=(commandlineArguments["aimRate"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["aimRate"]))) : (m_aimRate);
  m_aimFreq=(commandlineArguments["aimFreq"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["aimFreq"]))) : (m_aimFreq);
  m_prevAimReqRatio=(commandlineArguments["prevAimReqRatio"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["prevAimReqRatio"]))) : (m_prevAimReqRatio);
  m_useYawRate=(commandlineArguments["useYawRate"].size() != 0) ? (std::stoi(commandlineArguments["useYawRate"])==1) : (m_useYawRate);
  m_lowPassfactor=(commandlineArguments["lowPassfactor"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["lowPassfactor"]))) : (m_lowPassfactor);
  m_lowPassfactorAcc=(commandlineArguments["lowPassfactorAcc"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["lowPassfactorAcc"]))) : (m_lowPassfactorAcc);
  // sharp
  m_sharp=(commandlineArguments["useSharp"].size() != 0) ? (std::stoi(commandlineArguments["useSharp"])==1) : (m_sharp);
  m_nSharp=(commandlineArguments["nSharpPreviewPoints"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["nSharpPreviewPoints"]))) : (m_nSharp);
  m_K1=(commandlineArguments["sharpK1"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["sharpK1"]))) : (m_K1);
  m_Ky=(commandlineArguments["sharpKy"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["sharpKy"]))) : (m_Ky);
  m_C=(commandlineArguments["sharpBigC"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["sharpBigC"]))) : (m_C);
  m_c=(commandlineArguments["sharpSmallC"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["sharpSmallC"]))) : (m_c);
  // velocity control
  m_diffToBrakeVel=(commandlineArguments["diffToBrakeVel"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["diffToBrakeVel"]))) : (m_diffToBrakeVel);
  m_critDiff=(commandlineArguments["critDiff"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["critDiff"]))) : (m_critDiff);
  m_critDiff2=(commandlineArguments["critDiff2"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["critDiff2"]))) : (m_critDiff2);
  m_accFreq=(commandlineArguments["accFreq"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["accFreq"]))) : (m_accFreq);
  m_ayLimit=(commandlineArguments["ayLimit"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["ayLimit"]))) : (m_ayLimit);
  m_velocityLimit=(commandlineArguments["velocityLimit"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["velocityLimit"]))) : (m_velocityLimit);
  m_axLimitPositive=(commandlineArguments["axLimitPositive"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["axLimitPositive"]))) : (m_axLimitPositive);
  m_axLimitNegative=(commandlineArguments["axLimitNegative"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["axLimitNegative"]))) : (m_axLimitNegative);
  m_alwaysHeadingErrorDependent=(commandlineArguments["alwaysHeadingErrorDependent"].size() != 0) ? (std::stoi(commandlineArguments["alwaysHeadingErrorDependent"])==1) : (m_alwaysHeadingErrorDependent);
  m_headingErrorDependency=(commandlineArguments["headingErrorDependency"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["headingErrorDependency"]))) : (m_headingErrorDependency);
  m_curveDetectionAngleSlam=(commandlineArguments["curveDetectionAngleSlam"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["curveDetectionAngleSlam"]))) : (m_curveDetectionAngleSlam);
  m_curveDetectionPoints=(commandlineArguments["curveDetectionPoints"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["curveDetectionPoints"]))) : (m_curveDetectionPoints);
  m_curveDetectionPointsSlam=(commandlineArguments["curveDetectionPointsSlam"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["curveDetectionPointsSlam"]))) : (m_curveDetectionPointsSlam);
  m_maxPathLengthLocal=(commandlineArguments["maxPathLengthLocal"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["maxPathLengthLocal"]))) : (m_maxPathLengthLocal);
  // ....controller
  m_aimVel=(commandlineArguments["startAimVel"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["startAimVel"]))) : (m_aimVel);
  m_keepConstVel=(commandlineArguments["keepConstVel"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["keepConstVel"]))) : (m_keepConstVel);
  m_localVel=(commandlineArguments["localVel"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["localVel"]))) : (m_localVel);
  m_keepConstVelSlam=(commandlineArguments["keepConstVelSlam"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["keepConstVelSlam"]))) : (m_keepConstVelSlam);
  m_aKp=(commandlineArguments["aKp"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["aKp"]))) : (m_aKp);
  m_aKd=(commandlineArguments["aKd"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["aKd"]))) : (m_aKd);
  m_aKi=(commandlineArguments["aKi"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["aKi"]))) : (m_aKi);
  m_bKp=(commandlineArguments["bKp"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["bKp"]))) : (m_bKp);
  m_bKd=(commandlineArguments["bKd"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["bKd"]))) : (m_bKd);
  m_bKi=(commandlineArguments["bKi"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["bKi"]))) : (m_bKi);
  m_sKp=(commandlineArguments["sKp"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["sKp"]))) : (m_sKp);
  m_sKd=(commandlineArguments["sKd"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["sKd"]))) : (m_sKd);
  m_sKi=(commandlineArguments["sKi"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["sKi"]))) : (m_sKi);

  // curvature estimation
  m_polyFit=(commandlineArguments["usePolyFit"].size() != 0) ? (std::stoi(commandlineArguments["usePolyFit"])==1) : (false);
  m_step=(commandlineArguments["curvEstStepsize"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["curvEstStepsize"]))) : (m_step);
  m_polyDeg=(commandlineArguments["polynomialDegree"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["polynomialDegree"]))) : (m_polyDeg);
  m_pointsPerSegment=(commandlineArguments["pointsPerPolySegment"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["pointsPerPolySegment"]))) : (m_pointsPerSegment);
  m_segmentizePolyfit=(commandlineArguments["segmentizePolyfit"].size() != 0) ? (std::stoi(commandlineArguments["segmentizePolyfit"])==1) : (false);
  // vehicle specific
  m_wheelAngleLimit=(commandlineArguments["wheelAngleLimit"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["wheelAngleLimit"]))) : (m_wheelAngleLimit);
  m_frontToCog=(commandlineArguments["frontToCog"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["frontToCog"]))) : (m_frontToCog);

  //m_skidpadMode=(commandlineArguments["skidpadMode"].size() != 0) ? (std::stoi(commandlineArguments["skidpadMode"])==1) : (m_skidpadMode);
  /*//std::cout<<"Track set up with "<<commandlineArguments.size()<<" commandlineArguments: "<<std::endl;
  for (std::map<std::string, std::string >::iterator it = commandlineArguments.begin();it !=commandlineArguments.end();it++){
    //std::cout<<it->first<<" "<<it->second<<std::endl;
  }*/
  std::stringstream currentDateTime;
       time_t ttNow = time(0);
       tm * ptmNow;
       ptmNow = localtime(&ttNow);
       currentDateTime << 1900 + ptmNow->tm_year << "-";
       if (ptmNow->tm_mon < 9)
           currentDateTime << "0" << 1 + ptmNow->tm_mon << "-";
       else
           currentDateTime << (1 + ptmNow->tm_mon) << "-";
       if (ptmNow->tm_mday < 10)
           currentDateTime << "0" << ptmNow->tm_mday << "_";
       else
           currentDateTime <<  ptmNow->tm_mday << "_";
       if (ptmNow->tm_hour < 10)
           currentDateTime << "0" << ptmNow->tm_hour;
       else
           currentDateTime << ptmNow->tm_hour;
       if (ptmNow->tm_min < 10)
           currentDateTime << "0" << ptmNow->tm_min;
       else
           currentDateTime << ptmNow->tm_min;
       if (ptmNow->tm_sec < 10)
           currentDateTime << "0" << ptmNow->tm_sec;
       else
           currentDateTime << ptmNow->tm_sec;
  folderName="/opt/opendlv.data/"+currentDateTime.str()+"_velocityLogs";
  std::string command = "mkdir "+folderName;
  system(command.c_str());
}

void Track::tearDown()
{
}

void Track::receiveCombinedMessage(std::map<int,opendlv::logic::perception::GroundSurfaceArea> currentFrame, cluon::data::TimeStamp sampleTime){
  std::reverse_iterator<std::map<int,opendlv::logic::perception::GroundSurfaceArea>::iterator> it;
  it = currentFrame.rbegin();
  int I = 0;
  Eigen::MatrixXf localPath(currentFrame.size()*2,2);
  while(it != currentFrame.rend()){
    auto surfaceArea = it->second;
    float x1 = surfaceArea.x1(); //Unpack message
    float y1 = surfaceArea.y1();
    float x2 = surfaceArea.x2();
    float y2 = surfaceArea.y2();
    float x3 = surfaceArea.x3();
    float y3 = surfaceArea.y3();
    float x4 = surfaceArea.x4();
    float y4 = surfaceArea.y4();

    localPath(2*I,0)=(x1+x2)/2.0f;
    localPath(2*I,1)=(y1+y2)/2.0f;
    localPath(2*I+1,0)=(x3+x4)/2.0f;
    localPath(2*I+1,1)=(y3+y4)/2.0f;
    it++;
    I++;
  }
  Track::run(localPath, sampleTime);
} // End of recieveCombinedMessage

void Track::nextContainer(cluon::data::Envelope &a_container)
{
  /*if (a_container.dataType() == opendlv::proxy::GroundSpeedReading::ID()) {
    std::unique_lock<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
    auto groundSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(a_container));
    m_groundSpeed = groundSpeed.groundSpeed();
  }*/
  if (a_container.dataType() == opendlv::proxy::GroundSpeedReading::ID()) {
    std::unique_lock<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
    auto vehicleSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(a_container));
    if (m_speedId2>0) {
      if(a_container.senderStamp()==m_speedId1){
        m_groundSpeedReadingLeft = vehicleSpeed.groundSpeed();
      } else if (a_container.senderStamp()==m_speedId2){
        m_groundSpeedReadingRight = vehicleSpeed.groundSpeed();
      }
      m_groundSpeed = (m_groundSpeedReadingLeft + m_groundSpeedReadingRight)*0.5f;
    }
    else {
      m_groundSpeed = vehicleSpeed.groundSpeed();
    }
  }
  else if (a_container.dataType() == opendlv::proxy::AngularVelocityReading::ID()) {
    std::lock_guard<std::mutex> lockYaw(m_yawMutex);
    auto yawRate = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(std::move(a_container));
    m_yawRate = yawRate.angularVelocityZ();
  }
  else if (a_container.dataType() == opendlv::logic::perception::GroundSurfaceProperty::ID()) {
    m_STOP = true;
  }
  else if (!m_paramsUpdated) {
    if (a_container.dataType() == opendlv::logic::perception::ObjectDirection::ID()) {
      m_slamActivated = true;
    }
  }
}
bool Track::slamParams()
{
  //std::cout<<"SLAM ACTIVATEEEEEEEEEEEEEEEEEEEEEEEEED!!!"<<std::endl;
  m_keepConstVel = m_keepConstVelSlam;
  m_steerRate = m_steerRateSlam;
  m_minPrevDist = m_minPrevDistSlam;
  m_previewTime = m_previewTimeSlam;
  return true;
}
void Track::run(Eigen::MatrixXf localPath, cluon::data::TimeStamp sampleTime){
  if (m_slamActivated && !m_paramsUpdated) {
    m_paramsUpdated = slamParams();
  }

  float headingRequest;
  float distanceToAimPoint;
  float accelerationRequest;
  float groundSpeedCopy;
  {
    std::unique_lock<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
    groundSpeedCopy = m_groundSpeed;
  }

  localPath = pathProcessing(localPath);

  if (m_noPath) {
    headingRequest=m_prevHeadingRequest;
    distanceToAimPoint=1.0f;
  }
  else if (m_sharp && !m_onePoint) {
    float previewDistance = std::abs(groundSpeedCopy)*m_preview;
    float pathLength=localPath.row(0).norm();
    if(localPath.rows()>1){
      for (int i = 0; i < localPath.rows()-1; i++) {
        pathLength+=(localPath.row(i+1)-localPath.row(i)).norm();
      }
    }
    if (previewDistance>pathLength) {
      previewDistance = pathLength;
    }
    headingRequest = Track::driverModelSharp(localPath, previewDistance);
    distanceToAimPoint = 3.0f; //TODO only for plot
  }
  else{
  curveDetectionNoSlam(localPath);
  auto steering = Track::driverModelSteering(localPath, groundSpeedCopy);
  headingRequest = std::get<0>(steering);
  distanceToAimPoint = std::get<1>(steering);
  }
  accelerationRequest = Track::driverModelVelocity(localPath, groundSpeedCopy, headingRequest);
//std::cout<<"headingRequest: "<<headingRequest<<std::endl;
  { /*---SEND---*/
    std::unique_lock<std::mutex> lockSend(m_sendMutex);
    opendlv::logic::action::AimPoint steer;
    steer.azimuthAngle(headingRequest);
    steer.distance(distanceToAimPoint);
    m_od4.send(steer, sampleTime, m_senderStamp);

    if (accelerationRequest >= 0.0f) {
      opendlv::proxy::GroundAccelerationRequest acc;
      acc.groundAcceleration(accelerationRequest);
      m_od4.send(acc, sampleTime, m_senderStamp);
    }
    else if(accelerationRequest < 0.0f){
      opendlv::proxy::GroundDecelerationRequest dec;
      dec.groundDeceleration(-accelerationRequest);
      m_od4.send(dec, sampleTime, m_senderStamp);
    }

    if(m_STOP && groundSpeedCopy<0.1f){ //TODO: make sure it works
      opendlv::proxy::SwitchStateReading message;
      message.state(1);
      m_od4BB.send(message,sampleTime, 1403);
    }
  } //end mutex scope
}//end run

Eigen::VectorXf Track::curveFit(Eigen::MatrixXf matrix)
{
  Eigen::VectorXf a;
  Eigen::MatrixXf M;
  Eigen::VectorXf b = Eigen::VectorXf::Zero(m_polyDeg+1);
  M.resize(m_polyDeg+1,m_polyDeg+1);
  std::vector<float> v(2*m_polyDeg);
  std::fill(v.begin(), v.end(), 0.0f);
  for (uint32_t q = 0; q < matrix.rows(); q++) {
    for (uint32_t t = 0; t < v.size(); t++) {
      v[t] += powf(matrix(q,0),t+1);
      if (t<b.size()) {
        b(t) += matrix(q,1)*powf(matrix(q,0),t);
      }
    }
  }
  int c = 0;
  for (uint32_t q = 0; q < M.rows(); q++) {
    for (uint32_t t = 0; t < M.cols(); t++) {
      if (q==0 && t==0) {
        M(q,t)=matrix.rows();
      }
      else if (q==0) {
        M(q,t)=v[t-1];
      }
      else
      {
        M(q,t)=v[t+c];
      }
    }
    if (q>0) {
      c++;
    }
  }
  a = M.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  /*//std::cout << "Here is the matrix M:\n" << M << std::endl;
  //std::cout << "Here is the right hand side b:\n" << b << std::endl;
  //std::cout << "The least-squares solution is:\n"
      << a << std::endl;
  //std::cout<<"localPath:\n "<<matrix<<std::endl;*/
  return a;
}

Eigen::MatrixXf Track::placeEquidistantPoints(Eigen::MatrixXf oldPathPoints, bool nEqPointsIsKnown, int nEqPoints, float eqDistance)
{
// Places linearly equidistant points along a sequence of points.
// If nEqPoints is known it will not use the input value for eqDistance, and instead calculate a suitable value.
// If nEqPoints is not known it will not use the input value for nEqPoints, and instead calculate a suitable value.

  int nOld = oldPathPoints.rows();
  // Full path length, and save lengths of individual segments
  float pathLength = 0;
  Eigen::MatrixXf segLength(nOld-1,1);
  for(int i = 0; i < nOld-1; i = i+1)
  {
    segLength(i) = (oldPathPoints.row(i+1)-oldPathPoints.row(i)).norm();
    pathLength = pathLength + segLength(i);
  }

  if(nEqPointsIsKnown)
  {
    // Calculate equal subdistances
    eqDistance = pathLength/(nEqPoints-1);
  }
  else
  {
    //Calculate how many points will fit
    nEqPoints = static_cast<int>(std::ceil(static_cast<double>(pathLength)/static_cast<double>(eqDistance))+1.0);
  }
  // The latest point that you placed
  Eigen::MatrixXf latestNewPointCoords = oldPathPoints.row(0);
  // The latest path point that you passed
  int latestOldPathPointIndex = 0;
  // How long is left of the current segment
  float remainderOfSeg = segLength(0);
  // The new list of linearly equidistant points
  Eigen::MatrixXf newPathPoints(nEqPoints,2);
  // The first new point should be at the same place as the first old point
  newPathPoints.row(0) = latestNewPointCoords;
  // A temporary vector
  Eigen::MatrixXf vec(1,2);
  // Temporary distances
  float distanceToGoFromLatestPassedPoint, lengthOfNextSeg;
  // Start stepping through the given path
  for(int i = 1; i < nEqPoints-1; i = i+1)
  {
    // If the next new point should be in the segment you are currently in, simply place it.
    if(remainderOfSeg > eqDistance)
    {
      vec = oldPathPoints.row(latestOldPathPointIndex+1)-latestNewPointCoords;
      latestNewPointCoords = latestNewPointCoords + (eqDistance/remainderOfSeg)*vec;
    }
    else // If you need to go to the next segment, keep in mind which old points you pass and how long distance you have left to go.
    {
      latestOldPathPointIndex = latestOldPathPointIndex+1;
      distanceToGoFromLatestPassedPoint = eqDistance-remainderOfSeg;
      lengthOfNextSeg = segLength(latestOldPathPointIndex);

      while(distanceToGoFromLatestPassedPoint > lengthOfNextSeg)
      {
        latestOldPathPointIndex = latestOldPathPointIndex+1;
        distanceToGoFromLatestPassedPoint = distanceToGoFromLatestPassedPoint - lengthOfNextSeg;
        lengthOfNextSeg = segLength(latestOldPathPointIndex);
      } // End of while

      latestNewPointCoords = oldPathPoints.row(latestOldPathPointIndex);
      vec = oldPathPoints.row(latestOldPathPointIndex+1)-latestNewPointCoords;
      latestNewPointCoords = latestNewPointCoords + (distanceToGoFromLatestPassedPoint/segLength(latestOldPathPointIndex))*vec;
    } // End of else
    // In every case, save the point you just placed and check how much of that segment is left.
    newPathPoints.row(i) = latestNewPointCoords;
    remainderOfSeg = (oldPathPoints.row(latestOldPathPointIndex+1)-latestNewPointCoords).norm();
  } // End of for
  // The last point should be at the same place as the last cone.
  newPathPoints.row(nEqPoints-1) = oldPathPoints.row(nOld-1);

  return newPathPoints;
} // End of placeEquidistantPoints

Eigen::MatrixXf Track::orderCones(Eigen::MatrixXf localPath)
{
  // (A function from DetectConeLane)

  // Input: Cone and vehicle positions in the same coordinate system
  // Output: The localPath in order
  int nCones = localPath.rows();
  Eigen::MatrixXf current(1,2);
  current << -3,0;
  Eigen::ArrayXXi found(nCones,1);
  found.fill(-1);
  Eigen::MatrixXf orderedCones(nCones,2);
  float shortestDist;
  float tmpDist;
  int closestConeIndex = 0;

  // The first chosen cone is the one closest to the vehicle. After that it continues with the closest neighbour
  for(int i = 0; i < nCones; i = i+1)
  {

    shortestDist = std::numeric_limits<float>::infinity();
    // Find closest cone to the last chosen cone
    for(int j = 0; j < nCones; j = j+1)
    {
      if(!((found==j).any()))
      {
        tmpDist = (current-localPath.row(j)).norm();
        if(tmpDist < shortestDist)
        {
          shortestDist = tmpDist;
          closestConeIndex = j;
        } // End of if
      } // End of if
    } // End of for

    found(i) = closestConeIndex;
    current = localPath.row(closestConeIndex);
  } // End of for
  // Rearrange localPath to have the order of found
  for(int i = 0; i < nCones; i = i+1)
  {
    orderedCones.row(i) = localPath.row(found(i));
  } // End of for
  return orderedCones;
} // End of orderCones

float Track::driverModelSharp(Eigen::MatrixXf localPath, float previewDistance){
  float headingRequest;
  int n = m_nSharp;
  if (localPath.rows()>2) { //TODO: does not work for first lap??
    float sectionLength = previewDistance/n;
    std::vector<float> error(n);
    Eigen::MatrixXf::Index idx;
    float xMax = localPath.col(0).maxCoeff(&idx);
    error[0]=localPath(0,1);
    int k = 1;
    float sumPoints = 0.0f;
    for (int i = 0; i < localPath.rows()-1 ; i++) {
    sumPoints += (localPath.row(i+1)-localPath.row(i)).norm();
      if (sumPoints>=k*sectionLength) {
        if(k*sectionLength<xMax){
          error[k] = localPath(i,1);
          k++;
        }
        else{
          error[k] = localPath(idx,1);
          k++;
        }
      }
      if(k>n-1){
        break;
      }
    }
    if(k<n-2){
      error[n-1] = localPath(localPath.rows()-1,1);
    }

    float ey = atan2f(localPath(1,1)-localPath(0,1),localPath(1,0)-localPath(0,0));

    float errorSum = 0.0f;
    float j=0.0f;
    for (int i = 1; i < n; i++) {
      errorSum += error[i]*1.0f/expf(j)*m_C;
      j+=m_c;
    }

    headingRequest = m_Ky*ey + m_K1*error[1] + errorSum;
    if (headingRequest>=0) {
      headingRequest = std::min(headingRequest,m_wheelAngleLimit*m_PI/180.0f);
    } else {
      headingRequest = std::max(headingRequest,-m_wheelAngleLimit*m_PI/180.0f);
    }
  }
  else {
    headingRequest = m_prevHeadingRequest;
  }
  m_prevHeadingRequest = headingRequest;
  return headingRequest;
}


std::tuple<float, float> Track::driverModelSteering(Eigen::MatrixXf localPath, float groundSpeedCopy) {
  float headingRequest;
  float angleToAimPoint = 0.0f;
  float distanceToAimPoint;
  Eigen::Vector2f aimPoint(2);
  bool noPath = false;
  int count = 0;
  if (m_moveOrigin) {
    // Move localPath to front wheel axis
    Eigen::MatrixXf foo = Eigen::MatrixXf::Zero(localPath.rows(),2);
    foo.col(0).fill(m_frontToCog);
    localPath = localPath-foo;

    // Remove negative path points
    if (localPath(0,0)<0.0f && localPath.rows()>0) {
      while (localPath(count,0)<0.0f){
        count++;
        if (count>localPath.rows()-1) {
          noPath = true;
          //std::cout<<"whole path removed"<<std::endl;
          break;
        }
      }
    }
  }
  if(!noPath){
    Eigen::MatrixXf localPathTmp;
    if (count>0) {
      localPathTmp = localPath.bottomRows(localPath.rows()-count);
      localPath.resize(localPath.rows()-count,2);
      localPath = localPathTmp;
    }
    //std::cout<<"localPath:\n "<<localPath<<std::endl;
    if (m_curveFitPath) {
      localPathTmp = Eigen::MatrixXf::Zero(localPath.rows()+1,2); //Begin path at 0,0
      localPathTmp.bottomRows(localPath.rows())=localPath;
      float sumPoints = localPathTmp.row(0).norm(); //TODO: if above is used, this is same as sumPoints=0;
      int k=0;
      while (m_curveFitDistance >= sumPoints && k < localPathTmp.rows()-1) {
        sumPoints += (localPathTmp.row(k+1)-localPathTmp.row(k)).norm();
        k++;
        if (k < localPathTmp.rows()-1) {
          if (localPathTmp(k+1,0)<localPathTmp(k,0)) {
            break;
          }
        }
      }
    }
    if (m_curveFitPath && localPathTmp.rows()>10) {
      localPath = localPathTmp;
      localPathTmp = Eigen::MatrixXf::Zero(localPath.rows()+1,2); //Begin path at 0,0
      localPathTmp.bottomRows(localPath.rows())=localPath;
      localPath = localPathTmp;
      float sumPoints = localPath.row(0).norm(); //TODO: if above is used, this is same as sumPoints=0;
      int k=0;
      while (m_curveFitDistance >= sumPoints && k < localPath.rows()-1) {
        sumPoints += (localPath.row(k+1)-localPath.row(k)).norm();
        k++;
        if (k < localPath.rows()-1) {
          if (localPath(k+1,0)<localPath(k,0)) {
            break;
          }
        }
      }
      localPath = localPath.topRows(k);
      if (localPath.row(1).norm()>m_distanceBetweenPoints) { //add more points between car and first point (assuming first point is zero)
        int addPoints = static_cast<int>(floorf(localPath.row(1).norm()/m_distanceBetweenPoints));
        localPathTmp.resize(localPath.rows()+addPoints-1,2);
        localPathTmp.bottomRows(localPath.rows()-1)=localPath.bottomRows(localPath.rows()-1);
        for (int i = 0; i < addPoints; i++) {
          localPathTmp.row(i)=m_distanceBetweenPoints*(localPath.row(1)/(localPath.row(1).norm()))*i;
          //std::cout<<"point added: "<< localPathTmp.row(i)<<std::endl;
        }
      }
      //std::cout<<"localPathTmp:\n "<<localPathTmp<<std::endl;
      float xNorm = localPath(0,0); //TODO: no need to normalize if starting at 0,0
      float yNorm = localPath(0,1);
      localPath.col(0)=localPath.col(0).array()-xNorm;
      localPath.col(1)=localPath.col(1).array()-yNorm;
      Eigen::VectorXf a = curveFit(localPath);
      //localPath = localPathTmp;
      for (uint32_t i = 0; i < localPath.rows(); i++) {
        for (uint32_t j = 0; j < a.size(); j++) {
          if (j==0) {
            localPath(i,1) = 0.0f;
          }
          localPath(i,1) += a(j)*powf(localPath(i,0),j);
        }
      }
      localPath.col(0)=localPath.col(0).array()+xNorm;
      localPath.col(1)=localPath.col(1).array()+yNorm;
      //std::cout<<"New localPath:\n "<<localPath<<std::endl;
      /*std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
      cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
      opendlv::logic::perception::GroundSurfaceArea plot;
      plot.x1(a(0));
      plot.x2(a(1));
      plot.x3(a(2));
      if (a.size()>3) {
        plot.x4(a(3));
      }
      else{
        plot.x4(0.0f);
      }
      plot.y1(xNorm);
      plot.y2(yNorm);
      plot.y3(localPath(localPath.rows()-1,0)-xNorm);
      m_od4.send(plot, sampleTime, m_senderStamp);*/
    }
  }

  m_steerTockDt = std::chrono::system_clock::now();
  std::chrono::duration<float> DT = m_steerTockDt-m_steerTickDt;
  float dt = (DT.count()<1.0f) ? (DT.count()) : (0.1f); // Avoid large DT's to give high control outputs

  if (!noPath) {
    // Calculate the distance between vehicle and aimpoint;
    float previewDistance=0.0f;
    if (m_constPrevDist>=0) {
      previewDistance=m_constPrevDist;
    }
    else{
      previewDistance = std::max(std::abs(groundSpeedCopy)*m_preview,m_minPreview);
    }
    float sumPoints;
    // Sum the distance between all path points until passing previewDistance
    // or reaching end of path
    int k=0;
    if (!m_calcDtaOnX) {
      sumPoints = localPath.row(0).norm();
      while (previewDistance >= sumPoints && k < localPath.rows()-1) {
        sumPoints += (localPath.row(k+1)-localPath.row(k)).norm();
        k++;
      }
    }
    else{
      sumPoints = localPath(0,0);
      while (previewDistance >= sumPoints && k < localPath.rows()-1) {
        sumPoints += (localPath(k+1,0)-localPath(k,0));
        k++;
        if ((localPath(k+1,0)-localPath(k,0)) < 0.0f) {
          break;
        }
      }
    }

    if (sumPoints >= previewDistance) { // it means that the path is longer than previewDistance
      float distanceP1P2, overshoot, distanceP1AimPoint;
      if (k > 0) {// then the aimpoint will be placed after the first path element
        k--;
        // Distance between last two considered path elements P1 P2 where P2 is the overshoot element.
        distanceP1P2 = (localPath.row(k+1)-localPath.row(k)).norm();
        // Difference between distance to aimpoint and summed points.
        overshoot = sumPoints - previewDistance;
        // Distance between next to last considered path element P1 and aimpoint
        distanceP1AimPoint = distanceP1P2 - overshoot;
        // Linear interpolation
        aimPoint = (localPath.row(k+1)-localPath.row(k))*(distanceP1AimPoint/distanceP1P2) + localPath.row(k);
      }
      else {// Place aimpoint on first path element
        aimPoint = localPath.row(0);
      }
    }
    // If the path is too short, place aimpoint at the last path element
    else {
      aimPoint = localPath.row(localPath.rows()-1);
    }

    m_aimClock+=DT.count();
    if (m_aimClock>(1.0f/m_aimFreq)) {
      m_aimClock = 0.0f;
      // Angle to aimpoint
      angleToAimPoint = atan2f(aimPoint(1),aimPoint(0));
    }
    else{
      angleToAimPoint=m_prevAngleToAimPoint;
    }

    if (std::abs(angleToAimPoint-m_prevAngleToAimPoint)/dt>(m_aimRate*m_PI/180.0)){
      if (angleToAimPoint > m_prevAngleToAimPoint) {
        angleToAimPoint = dt*m_aimRate*m_PI/180.0f + m_prevAngleToAimPoint;
      }
      else{
        angleToAimPoint = -dt*m_aimRate*m_PI/180.0f + m_prevAngleToAimPoint;
      }
    }
    if (m_prevAimReqRatio > 0.0f) {
      angleToAimPoint = m_prevAimReqRatio*m_prevAngleToAimPoint + (1.0f-m_prevAimReqRatio)*angleToAimPoint;
    }
    if (m_lowPassfactor>0) {
      angleToAimPoint = lowPass(m_lowPassfactor, m_prevAngleToAimPoint, angleToAimPoint);
    }

    float e = angleToAimPoint;
    float ed=0.0f;
    if (m_useYawRate) {
      ed = m_yawRate;
    }
    else {
      ed = (e-m_sEPrev)/dt;
    }
    if (groundSpeedCopy<=0.0f) {
      m_sEi=0.0f;
    }
    else{
      m_sEi+=e*dt;
    }
    headingRequest = e*m_sKp+ed*m_sKd+m_sEi*m_sKi;
    m_sEPrev=e;

    // Limit heading request due to physical limitations
    if (headingRequest>=0) {
      headingRequest = std::min(headingRequest,m_wheelAngleLimit*m_PI/180.0f);
    } else {
      headingRequest = std::max(headingRequest,-m_wheelAngleLimit*m_PI/180.0f);
    }

    distanceToAimPoint=aimPoint.norm();
  }
  else{
    headingRequest=m_prevHeadingRequest;
    distanceToAimPoint = 3.0f; //TODO: distanceToAimPoint is only used for plot, everywhere.
  }


  if (m_inRightCurve && headingRequest>-m_curveSteerAmpLim*m_PI/180.0f) {
    //std::cout<<"headingRequest limited from: "<<headingRequest;
    headingRequest = -m_curveSteerAmpLim*m_PI/180.0f;
    //std::cout<<" to: "<<headingRequest<<std::endl;
  }
  else if (m_inLeftCurve && headingRequest<m_curveSteerAmpLim*m_PI/180.0f) {
    //std::cout<<"headingRequest limited from: "<<headingRequest;
    headingRequest = m_curveSteerAmpLim*m_PI/180.0f;
    //std::cout<<" to: "<<headingRequest<<std::endl;
  }

  if (m_prevReqRatio > 0.0f) {
    //std::cout<<"headingRequest ratio used from: "<<headingRequest;
    headingRequest = m_prevReqRatio*m_prevHeadingRequest + (1.0f-m_prevReqRatio)*headingRequest;
    //std::cout<<" to: "<<headingRequest<<std::endl;
  }

  if (std::abs(headingRequest-m_prevHeadingRequest)/dt>(m_steerRate*m_PI/180.0)){
    if (headingRequest > m_prevHeadingRequest) { //leftTurn
      headingRequest = dt*m_steerRate*m_PI/180.0f + m_prevHeadingRequest;
    }
    else{
      headingRequest = -dt*m_steerRate*m_PI/180.0f + m_prevHeadingRequest;
    }
  }
  m_prevAngleToAimPoint = angleToAimPoint;
  m_prevHeadingRequest=headingRequest;
  m_steerTickDt = std::chrono::system_clock::now();
  return std::make_tuple(headingRequest,distanceToAimPoint);
}

float Track::driverModelVelocity(Eigen::MatrixXf localPath, float groundSpeedCopy, float headingRequest){
  float accelerationRequest = 0.0f;
  Eigen::VectorXf speedProfile;
  std::vector<float> curveRadii;
  int step = m_step;
  float pathLength=0.0f;//localPath.row(0).norm();
  if(localPath.rows()>1){
    for (int i = 0; i < localPath.rows()-1; i++) {
      pathLength+=(localPath.row(i+1)-localPath.row(i)).norm();
    }
  }
  bool slam;
  if (pathLength>m_maxPathLengthLocal) {
    slam=true;
    //std::cout<<"SLAM"<<std::endl;
  }
  else{
    slam=false;
    //std::cout<<"LOCAL"<<std::endl;
  }

  m_tockDt = std::chrono::system_clock::now();
  std::chrono::duration<float> DT = m_tockDt-m_tickDt;
  float dt = (DT.count()<1.0f) ? (DT.count()) : (0.1f); // Avoid large DT's to give high control outputs

  if ((!m_STOP) && slam && m_slamActivated && (m_keepConstVel<0.0f)){
    m_preview=m_previewTimeSlam;
    m_minPreview = m_minPrevDistSlam;
    m_specCase = false;
    m_aimVelSet = false;
    m_case = 0;
    // Caluclate curvature of path
      if (m_polyFit){
        step = 0;
        curveRadii = curvaturePolyFit(localPath);
      }
      else{
        while (localPath.rows()-(2*step)<=0 && step > 0){ //This makes sure that step is as big as possible (up to limit)
          step--;
        }
        curveRadii = curvatureTriCircle(localPath,step);
      }
    float headingError = std::abs(headingRequest)/(m_wheelAngleLimit*m_PI/180.0f);
    // Set velocity candidate based on expected lateral acceleration
    float averageSpeed=0.0f;
    int n=0;
    speedProfile.resize(curveRadii.size());
    for (uint32_t k = 0; k < curveRadii.size(); k++){
      speedProfile(k) = std::min(sqrtf(m_ayLimit*curveRadii[k]),m_velocityLimit);//*(1.0f-headingError*headingErrorDependency);
      if (speedProfile(k)<m_localVel) {
        speedProfile(k)=m_localVel;
      }
      if (n<10) {
        averageSpeed+=speedProfile(k);
        n++;
      }
    }
    averageSpeed=averageSpeed/n;

    Eigen::MatrixXf P;
    int aIdx=-1;
    int curveDetectionPoints = (static_cast<int>(curveRadii.size())<m_curveDetectionPointsSlam) ? (curveRadii.size()-1):(m_curveDetectionPointsSlam);
    for (int k = step; k < localPath.rows()-step-curveDetectionPoints; k++) {
      P=localPath.row(k+curveDetectionPoints)-localPath.row(k);
      float angle = atan2f(P(1),P(0));
      if (std::abs(angle)>m_curveDetectionAngleSlam) {
        aIdx=k-step;
        break;
      }
    }

    if (aIdx>=0) {
      m_minRadius=100000.0f;
      int rIdx=0;
      for (uint32_t k = 0; k < curveRadii.size(); k++) {
        if (curveRadii[k]<m_minRadius) {
          m_minRadius=curveRadii[k];
          rIdx=k;
        }
      }

      if (aIdx==0 && (m_apexRadius-m_minRadius)>0) {
        m_apexRadius = m_minRadius;
      }
      /*std::chrono::system_clock::time_point tp = std::chrono::system_clock::now(); //draw curve start point and end of angle point
      cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
      opendlv::body::ActuatorInfo plot;
      plot.x(localPath(aIdx,0));
      plot.y(localPath(aIdx,1));
      plot.z(localPath(aIdx+curveDetectionPoints,0));
      plot.minValue(localPath(aIdx+curveDetectionPoints,1));
      m_od4.send(plot, sampleTime, 55);*/

      speedProfile(aIdx)=speedProfile(rIdx);
    }
    //std::cout<<"speedProfile: \n"<<speedProfile<<std::endl;
    float tb;
    float tv=0.0f;
    float s=0.0f;
    float diff;
    float critDiff=100000.0f;
    int i;
    int idx = 0;

    for (int k=0; k<step; k++){
      s+=(localPath.row(k+1)-localPath.row(k)).norm();
    }
    for (i=0; i<speedProfile.size(); i++){
      s+=(localPath.row(i+step+1)-localPath.row(i+step)).norm();
      tb = (speedProfile(i)-groundSpeedCopy)/(m_axLimitNegative);//time to reach velocity if braking max
      tv = s/groundSpeedCopy;
      diff = tv-tb; // when we need to brake, if =0 we need to brake now, if <0 we are braking too late
      if(diff<critDiff && tb > 0){ // save critical brake point
        critDiff=diff;
        idx=i;
      }
    }
      m_tv=tv;
      m_brakeInXSeconds=critDiff;
      m_critVel=speedProfile(idx);
    /*---------------State selection---------------*/
    /*BRAKING STATE*/
    if(critDiff<=m_critDiff){ //braking is critical
      if (((groundSpeedCopy-m_critVel)>m_diffToBrakeVel) || m_brakingState) {
        if (m_critVel<m_aimVel) {
          m_aimVel = m_critVel;
        }
        if (m_prevState) {
          resetPID();
        }
        m_brakingState = true;
        m_accelerationState = false;
        m_apexRadius=100000.0f;
      }
    }
    if (m_brakingState) {
      if (groundSpeedCopy>m_aimVel) {
        accelerationRequest = bPID(dt, groundSpeedCopy);
        //std::cout<<"B"<<std::endl;
      }
      else{
        m_brakingState = false;
      }
    }
    /*ACCELERATION STATE*/
    if (!m_brakingState){
      if (!m_accelerationState) {
        m_accClock = 0.0f;
        if (aIdx==0 && (m_inLeftCurve || m_inRightCurve)) {
          //m_aimVel = (speedProfile(0)*(1.0f-headingError*m_headingErrorDependency)>groundSpeedCopy) ? (speedProfile(0)*(1.0f-headingError*m_headingErrorDependency)) : (groundSpeedCopy);
          m_aimVel = (averageSpeed*(1.0f-headingError*m_headingErrorDependency)>groundSpeedCopy) ? (averageSpeed*(1.0f-headingError*m_headingErrorDependency)) : (groundSpeedCopy);
          //std::cout<<"Restricted by heading"<<std::endl;
        }
        else{
          if (m_alwaysHeadingErrorDependent) {
            m_aimVel = (averageSpeed*(1.0f-headingError*m_headingErrorDependency)>groundSpeedCopy) ? (averageSpeed*(1.0f-headingError*m_headingErrorDependency)) : (groundSpeedCopy);
          }
          else{
            m_aimVel = averageSpeed;//m_velocityLimit;//speedProfile(0);
          }
        }

        if (critDiff<m_critDiff2) {
          m_aimVel=groundSpeedCopy;
          //std::cout<<"DONT ACCELERATE"<<std::endl;
        }
        //std::cout<<"AIM VEL UPDATED 1"<<" aIdx: "<<aIdx<<std::endl;
        if (!m_prevState) {
          resetPID();
        }
      }
      if (aIdx>=0 && aIdx<5) {
        if ((m_minRadius-m_apexRadius)<1.0f) { //before apex
          if (m_critVel<m_aimVel) {
            m_aimVel = m_critVel;
          }
          //std::cout<<"Restricted by curve 1"<<std::endl;
        }
        else{//Apex passed
          m_apexRadius=0.0f;
        }
      }
      m_accelerationState = true;
    }

    if (m_accelerationState) {
      m_accClock+=DT.count();
      if (m_accClock>(1.0f/m_accFreq)) {
        m_accClock = 0.0f;
        if (aIdx>=0 && aIdx<5 && (m_minRadius-m_apexRadius)<1.0f) {
          if (m_critVel<m_aimVel) {
            m_aimVel = m_critVel;
          }
          //std::cout<<"Restricted by curve 2"<<std::endl;
        }
        else if (aIdx==0 && (m_inLeftCurve || m_inRightCurve)) {
          //m_aimVel = 5.0f;//(speedProfile(0)*(1.0f-headingError*m_headingErrorDependency)>groundSpeedCopy) ? (speedProfile(0)*(1.0f-headingError*m_headingErrorDependency)) : (groundSpeedCopy);
          m_aimVel = (averageSpeed*(1.0f-headingError*m_headingErrorDependency)>groundSpeedCopy) ? (averageSpeed*(1.0f-headingError*m_headingErrorDependency)) : (groundSpeedCopy);
          //std::cout<<"Restricted by heading"<<std::endl;
        }
        else{
          if (m_alwaysHeadingErrorDependent) {
            m_aimVel = (averageSpeed*(1.0f-headingError*m_headingErrorDependency)>groundSpeedCopy) ? (averageSpeed*(1.0f-headingError*m_headingErrorDependency)) : (groundSpeedCopy);
          }
          else{
            m_aimVel = averageSpeed;//m_velocityLimit;//speedProfile(0);
          }
        }
        if (critDiff<m_critDiff2) {
          m_aimVel=groundSpeedCopy;
          //std::cout<<"DONT ACCELERATE"<<std::endl;
        }
        //std::cout<<"AIM VEL UPDATED 2"<<" aIdx: "<<aIdx<<std::endl;
      }
      //std::cout<<"A"<<std::endl;
    }
  }
  /*SET SPECIAL CASES*/
  else{
    m_brakeInXSeconds-=DT.count();
    if (m_STOP) {
      m_aimVel=0.0f;
      m_brakingState=true;
      m_accelerationState=false;
      if (m_keepConstVel>0.0f) {
        m_specCase=false;
        m_keepConstVel = -1.0f;
      }
      if (!m_specCase) {
        resetPID();
        m_specCase=true;
      }
      //std::cout<<"S"<<std::endl;
    }
    else if(m_keepConstVel>0.0f){
      m_aimVel=m_keepConstVel;
      m_brakingState=false;
      m_accelerationState=true;
      m_preview=m_previewTime;
      m_minPreview=m_minPrevDist;
      //std::cout<<"K"<<std::endl;
      if (!m_specCase) {
        resetPID();
        m_specCase=true;
      }
    }
    else { //If slam is activated but running locally
      //std::cout<<"pathLength: "<<pathLength<<std::endl;
      if (m_brakeInXSeconds<=m_critDiff && groundSpeedCopy>m_critVel) { //If we need to brake now to reach saved critical velocity
        if (m_critVel<m_aimVel) {
          m_aimVel = m_critVel;
        }
        if (!(m_case==1)) {
          if (m_prevState) {
            resetPID();
          }
        }
        m_case=1;
        m_brakingState=false;
        //std::cout<<"C 1"<<std::endl;
      }
      else if(!(m_brakeInXSeconds<=m_critDiff) && groundSpeedCopy>m_critVel){ //If we don't need to brake yet, keep the same aim as before if braking, otherwise keep same velocity
        if (m_aimVel<groundSpeedCopy) {
          if (!(m_case==2)) {
            if (m_prevState) {
              resetPID();
            }
          }
          m_case=2;
          m_brakingState=false;
          //std::cout<<"C 2"<<std::endl;
        }
        else{
          if (!m_aimVelSet) {
            m_aimVel=groundSpeedCopy;
            m_aimVelSet=true;
          }
          if (!(m_case==3)) {
            if (!m_prevState) {
              resetPID();
            }
          }
          m_case=3;
          m_accelerationState=false;
          //std::cout<<"C 3"<<std::endl;
        }
      }
      else if(m_tv>1.0f && (m_case==0 || m_case==4)){ //If all upcoming velocities are higher than current, keep current for time m_tv
          m_tv-=DT.count();
          if (!m_aimVelSet) {
            m_aimVel=(groundSpeedCopy>m_localVel)?(groundSpeedCopy):(m_localVel);
            m_aimVelSet=true;
          }
          if (!(m_case==4)) {
            if (!m_prevState) {
              resetPID();
            }
          }
          m_case=4;
          m_accelerationState=false;
          //std::cout<<"C 4, m_tv: "<<m_tv<<std::endl;
        }
      else{
        if ((m_localVel-groundSpeedCopy)<-1.0f) {
          if (!(m_case==5)) {
            if (m_prevState) {
              resetPID();
            }
            m_aimVel=m_localVel;
          }
          m_case=5;
          m_brakingState=false;
          //std::cout<<"C 5"<<std::endl;
        }
        else{
          if (!(m_case==6)) {
            if (!m_prevState) {
              resetPID();
            }
            m_aimVel=m_localVel;
          }
          m_case=6;
          m_preview=m_previewTime;
          m_minPreview=m_minPrevDist;
          m_accelerationState=false;
          //std::cout<<"C 6"<<std::endl;
        }
      }
    }
  }

  if (m_brakingState || m_case==1 || m_case==2 || m_case==5) {
    accelerationRequest = bPID(dt, groundSpeedCopy);
    if (!m_prevState && m_lowPassfactorAcc>0) {
      accelerationRequest = lowPass(m_lowPassfactorAcc, m_prevAccelerationRequest, accelerationRequest);
    }
    m_prevState=false;
  }
  else if (m_accelerationState || m_case==3 || m_case==4 || m_case==6) {
    accelerationRequest = aPID(dt, groundSpeedCopy);
    if (m_prevState && m_lowPassfactorAcc>0) {
      accelerationRequest = lowPass(m_lowPassfactorAcc, m_prevAccelerationRequest, accelerationRequest);
    }
    m_prevState=true;
  }

  if (m_STOP) {
    if (accelerationRequest>0.0f) {
      accelerationRequest = 0.0f;
    }
  }
  m_prevAccelerationRequest=accelerationRequest;
  //std::cout<<m_aimVel<<"    |    "<<groundSpeedCopy<<"    |    "<<accelerationRequest<<std::endl;
  //std::cout<<"aim: "<<m_aimVel<<" acc: "<<accelerationRequest<<" V: "<<groundSpeedCopy<<std::endl;
  //std::cout<<"m_critVel: "<<m_critVel<<" m_aimVel: "<<m_aimVel<<std::endl;
  /* --write data to file-- */
  m_fullTime += DT.count();
  std::ofstream eiFile;
        eiFile.open(folderName+"/eiLog.txt",std::ios_base::app);
        eiFile<<m_ei<<std::endl;
        eiFile.close();
  std::ofstream accFile;
        accFile.open(folderName+"/accelerationLog.txt",std::ios_base::app);
        accFile<<accelerationRequest<<std::endl;
        accFile.close();
  std::ofstream timeFile;
              timeFile.open(folderName+"/timeLog.txt",std::ios_base::app);
              timeFile<<m_fullTime<<std::endl;
              timeFile.close();
  std::ofstream speedFile;
              speedFile.open(folderName+"/speedLog.txt",std::ios_base::app);
              speedFile<<groundSpeedCopy<<std::endl;
              speedFile.close();
  std::ofstream refSpeedFile;
              refSpeedFile.open(folderName+"/refSpeedLog.txt",std::ios_base::app);
              refSpeedFile<<m_aimVel<<std::endl;
              refSpeedFile.close();

  m_tickDt = std::chrono::system_clock::now();
  return accelerationRequest;
}

std::vector<float> Track::curvatureTriCircle(Eigen::MatrixXf localPath, int step){
  // Segmentize the path and calculate radius of segments
  // - First radius is calculated at path point "step"
  // - Last radius is calculated at end-step path point
  // Note! 3 points in a row gives infinate radius.
  std::vector<float>  curveRadii(localPath.rows()-(2*step));
  for (int k = 0; k < localPath.rows()-(2*step); k++) {
    // Choose three points and make a triangle with sides A(p1p2),B(p2p3),C(p1p3)
    float A = (localPath.row(k+step)-localPath.row(k)).norm();
    float B = (localPath.row(k+2*step)-localPath.row(k+step)).norm();
    float C = (localPath.row(k+2*step)-localPath.row(k)).norm();

    // sort side lengths as A >= B && B >= C
    if (A < B) {
        std::swap(A, B);
    }
    if (A < C) {
        std::swap(A, C);
    }
    if (B < C) {
        std::swap(B, C);
    }

    if (C-(A-B) <= 0) {
      //std::cout << "WARNING! The data are not side-lengths of a real triangle" << std::endl;
      curveRadii[k] = 10000; // Large radius instead of inf value will reach m_velocityLimit
    }
    else {
      // https://people.eecs.berkeley.edu/~wkahan/Triangle.pdf
      // Calculate triangle area
      float triangleArea = 0.25f*sqrtf((A+(B+C))*(C-(A-B))*(C+(A-B))*(A+(B-C)));
      // Calculate the radius of the circle that matches the points
      curveRadii[k] = (A*B*C)/(4*triangleArea);
    }
  }

  return curveRadii;
}

std::vector<float> Track::curvaturePolyFit(Eigen::MatrixXf localPath){
  //int n = m_polyDeg;
  int pointsPerSegment = m_pointsPerSegment;
  //int i,j;
  int segments,N;
  int k=0;
  bool BREAK=false;
  //int segmentcount = 0; //TODO: only for plot

  uint32_t l=0;
  Eigen::VectorXf dividedPathX(localPath.rows()); // TODO: This is now maximum possible size, which in some cases is unneccesary
  Eigen::VectorXf dividedPathY(localPath.rows());
  std::vector<Eigen::VectorXf> dividedPathsX;
  std::vector<Eigen::VectorXf> dividedPathsY;

  while (l < localPath.rows()-2){ //TODO improve this section
    while ((localPath(l,0) >= localPath(l+1,0)) && l < localPath.rows()-2){ // While X decreasing
      dividedPathX(k) = localPath(l,0);
      dividedPathY(k) = localPath(l,1);
      l++;
      k++;
        if (!((localPath(l,0) >= localPath(l+1,0)) && l < localPath.rows()-2)){ //If not decreasing, save this part of path
          dividedPathsX.push_back(dividedPathX.segment(0,k));
          dividedPathsY.push_back(dividedPathY.segment(0,k));
          k=0;
          if (!m_segmentizePolyfit) {
            BREAK = true;
          }
        }
    }
    while ((localPath(l,0) < localPath(l+1,0)) && l < localPath.rows()-2){ //While X increasing
      dividedPathX(k) = localPath(l,0);
      dividedPathY(k) = localPath(l,1);
      l++;
      k++;
        if (!((localPath(l,0) < localPath(l+1,0)) && l < localPath.rows()-2)){ // If not increasing, save this part of path
          dividedPathsX.push_back(dividedPathX.segment(0,k));
          dividedPathsY.push_back(dividedPathY.segment(0,k));;
          k=0;
          if (!m_segmentizePolyfit) {
            BREAK = true;
          }
        }
    }
    if (BREAK) {
      break;
    }
  }
  std::vector<float> curveRadii;
  std::vector<float> R;
  Eigen::VectorXf pathx;
  Eigen::VectorXf pathy;
  Eigen::VectorXf x;
  Eigen::VectorXf y;
  //Eigen::VectorXf a(n+1);
  int segmentBegin;
  int segmentLength = pointsPerSegment;
  for (uint32_t P=0; P<dividedPathsX.size(); P++) {
    pathx = dividedPathsX[P];
    pathy = dividedPathsY[P];
    N = pointsPerSegment; // number of path points per segment
    if (pathx.size()<N) {
      N = pathx.size();
    }
    if (m_segmentizePolyfit) {
      segments = pathx.size()/N; //number of segments
    }else{
      segments=1;
    }

    for (int p=0; p<segments; p++){
      if ((p<segments-1) || (!(segments*N<pathx.size())) ){
      segmentBegin = p*N;
      segmentLength = N;
      }
      if (segments*N<pathx.size() && p+1>=segments){ // Use all points in last segment, N+rest
       segmentBegin = p*N;
       segmentLength = N+pathx.size()-N*segments;
       N = segmentLength;
      }
      x = pathx.segment(segmentBegin,segmentLength).array()-pathx(segmentBegin);
      y = pathy.segment(segmentBegin,segmentLength).array()-pathy(segmentBegin);
      Eigen::MatrixXf matrix(x.size(),2);
      matrix.col(0)=x;
      matrix.col(1)=y;
      Eigen::VectorXf a = curveFit(matrix);

      R.resize(x.size()); // stores curvatures
      for(uint32_t m=0; m<R.size();m++){
        R[m] = 1/std::abs(2*a(2)+6*a(3)*x(m))/powf(1+powf(a(1)+2*a(2)*x(m)+3*a(3)*powf(x(m),2),2),1.5);
        /*if (R[m]<9.0f) {
          R[m]=9.0f;
        }*/
      }
      curveRadii.insert(curveRadii.end(), R.begin(), R.end());


      /*-------------TODO: remove ONLY FOR PLOT--------------*/
      /*if (segmentcount==0) {
        //std::cout<<"a0: "<<a(0)<<" a1: "<<a(1)<<" a2: "<<a(2)<<" a3: "<<a(3)<<std::endl;
        std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
        cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
        opendlv::logic::perception::GroundSurfaceArea plot;
        plot.x1(a(0));
        plot.x2(a(1));
        plot.x3(a(2));
        if (a.size()>3) {
          plot.x4(a(3));
        }
        else{
          plot.x4(0.0f);
        }
        plot.y1(pathx(segmentBegin));
        plot.y2(pathy(segmentBegin));
        plot.y3(x(x.size()-1));
        m_od4.send(plot, sampleTime, 11);
      }
      if (m_segmentizePolyfit) {
        if (segmentcount==1) {
          //std::cout<<"a0: "<<a(0)<<" a1: "<<a(1)<<" a2: "<<a(2)<<" a3: "<<a(3)<<std::endl;
          std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
          cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
          opendlv::logic::perception::GroundSurfaceArea plot;
          plot.x1(a(0));
          plot.x2(a(1));
          plot.x3(a(2));
          if (a.size()>3) {
            plot.x4(a(3));
          }
          else{
            plot.x4(0.0f);
          }
          plot.y1(pathx(segmentBegin));
          plot.y2(pathy(segmentBegin));
          plot.y3(x(x.size()-1));
          m_od4.send(plot, sampleTime, 22);
        }
        if (segmentcount==2) {
          //std::cout<<"a0: "<<a(0)<<" a1: "<<a(1)<<" a2: "<<a(2)<<" a3: "<<a(3)<<std::endl;
          std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
          cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
          opendlv::logic::perception::GroundSurfaceArea plot;
          plot.x1(a(0));
          plot.x2(a(1));
          plot.x3(a(2));
          if (a.size()>3) {
            plot.x4(a(3));
          }
          else{
            plot.x4(0.0f);
          }
          plot.y1(pathx(segmentBegin));
          plot.y2(pathy(segmentBegin));
          plot.y3(x(x.size()-1));
          m_od4.send(plot, sampleTime, 33);
        }
        if (segmentcount==3) {
          //std::cout<<"a0: "<<a(0)<<" a1: "<<a(1)<<" a2: "<<a(2)<<" a3: "<<a(3)<<std::endl;
          std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
          cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
          opendlv::logic::perception::GroundSurfaceArea plot;
          plot.x1(a(0));
          plot.x2(a(1));
          plot.x3(a(2));
          if (a.size()>3) {
            plot.x4(a(3));
          }
          else{
            plot.x4(0.0f);
          }
          plot.y1(pathx(segmentBegin));
          plot.y2(pathy(segmentBegin));
          plot.y3(x(x.size()-1));
          m_od4.send(plot, sampleTime, 44);
        }
        segmentcount++;
      }*/
    } // end p-loop
  } // end P-loop
  /*//std::cout<<"curveRadii: ";
  for (uint32_t i = 0; i < curveRadii.size(); i++) {
    //std::cout<<curveRadii[i]<<" ";
  }
  //std::cout<<"\n"<<std::endl;*/
  //TODO: poly fit misses to catch turns where x is decreasing, could work to switch y and x for these cases only..

  return curveRadii;
} // end curvaturePolyFit

void Track::curveDetectionNoSlam(Eigen::MatrixXf localPath)
{
  Eigen::MatrixXf P;
  int curveDetectionPoints = (static_cast<int>(localPath.rows())<m_curveDetectionPoints) ? (localPath.rows()-1):(m_curveDetectionPoints);
  P=localPath.row(curveDetectionPoints);
  float angle = atan2f(P(1),P(0));
  float initAngle;
  if (!m_onePoint) {
    P = localPath.row(1)-localPath.row(0);
    initAngle = atan2f(P(1),P(0));
  }
  else{
    initAngle = 0.0f;
  }
  //std::cout<<"angle: "<<angle<<std::endl;
  if (std::abs(angle)>m_curveDetectionAngle) {
    if (angle<0.0f && initAngle<=0.0f) {
      m_inRightCurve = true;
      m_inLeftCurve = false;
      //std::cout<<"in right curve"<<std::endl;
    }
    else if (angle>0.0f && initAngle>=0.0f){
      m_inLeftCurve = true;
      m_inRightCurve=false;
      //std::cout<<"in left curve"<<std::endl;
    }
  }
  else if (m_inLeftCurve && angle<m_curveExitAngleLim) {
    m_inLeftCurve=false;
    //std::cout<<"Exit curve"<<std::endl;
  }
  else if (m_inRightCurve && angle>-m_curveExitAngleLim) {
    m_inRightCurve=false;
  }
}

Eigen::MatrixXf Track::pathProcessing(Eigen::MatrixXf localPath)
{
  m_noPath=false;
  m_onePoint=false;
  //check for no cone signal
  if (localPath.rows()<=0 || (localPath.rows()==2 && (std::abs(localPath(0,0))<=0.00001f && std::abs(localPath(1,0))<=0.00001f && std::abs(localPath(0,1))<=0.00001f && std::abs(localPath(1,1))<=0.00001f))){
    //No cone
    m_noPath = true;
  }
  else{
    // Check for one cone signal
    if(localPath.rows()==2 && (std::abs(localPath(0,0))<=0.00001f && std::abs(localPath(0,1))<=0.00001f)){
      //One cone -> one point
      if (m_ignoreOnePoint) {
        if (m_prevHeadingRequest<0.0f) {
          if (atan2f(localPath(1,1),localPath(1,0)-m_frontToCog)>=m_prevHeadingRequest) {
            m_noPath = true;
            //std::cout<<"one point ignored, onepoint heading: "<<atan2f(localPath(1,1),localPath(1,0)-m_frontToCog)<<" prev heading: "<<m_prevHeadingRequest<<std::endl;
          }
          else{
            //std::cout<<"one point approved, onepoint heading: "<<atan2f(localPath(1,1),localPath(1,0)-m_frontToCog)<<" prev heading: "<<m_prevHeadingRequest<<std::endl;
            Eigen::MatrixXf localPathTmp = localPath.row(1);
            localPath.resize(1,2);
            localPath = localPathTmp;
            if (localPath(0,0)>0) {
              m_onePoint=true;
            }
            else{
              m_noPath=true;
            }
          }
        }
        else if (m_prevHeadingRequest>0.0f){
          if (atan2f(localPath(1,1),localPath(1,0)-m_frontToCog)<=m_prevHeadingRequest) {
            m_noPath = true;
            //std::cout<<"one point ignored, onepoint heading: "<<atan2f(localPath(1,1),localPath(1,0)-m_frontToCog)<<" prev heading: "<<m_prevHeadingRequest<<std::endl;
          }
          else{
            //std::cout<<"one point approved, onepoint heading: "<<atan2f(localPath(1,1),localPath(1,0)-m_frontToCog)<<" prev heading: "<<m_prevHeadingRequest<<std::endl;
            Eigen::MatrixXf localPathTmp = localPath.row(1);
            localPath.resize(1,2);
            localPath = localPathTmp;
            if (localPath(0,0)>0) {
              m_onePoint=true;
            }
            else{
              m_noPath=true;
            }
          }
        }
      }
      else{
        Eigen::MatrixXf localPathTmp = localPath.row(1);
        localPath.resize(1,2);
        localPath = localPathTmp;
        if (localPath(0,0)>0) {
          m_onePoint=true;
        }
        else{
          m_noPath=true;
        }
      }
    }
    else if(localPath.rows()==2 && (std::abs(localPath(1,0))<=0.00001f && std::abs(localPath(1,1))<=0.00001f)){
      //Two cones -> one point
      Eigen::MatrixXf localPathTmp = localPath.row(0);
      localPath.resize(1,2);
      localPath = localPathTmp;
      if (localPath(0,0)>0) {
        m_onePoint=true;
      }
      else{
        m_noPath=true;
      }
      //std::cout<<"One Point Signal recieved"<<std::endl;
    }
    if (!m_onePoint && !m_noPath) {
      // Order path
      if (m_orderPath) {
        localPath = orderCones(localPath);
      }
      // Remove negative path points
      int count = 0;
      if (localPath(0,0)<0.0f) {
        while (localPath(count,0)<0.0f){
          count++;
          if (count>localPath.rows()-2) {
            m_onePoint = true;
          }
          if (count>localPath.rows()-1) {
            m_noPath = true;
            m_onePoint = false;
            break;
          }
        }
      }
      if (!m_noPath) {
        if (count>0) {
          Eigen::MatrixXf localPathTmp = localPath.bottomRows(localPath.rows()-count);
          localPath.resize(localPath.rows()-count,2);
          localPath = localPathTmp;
        }
        if (!m_onePoint) {
          //Place equidistant points
          localPath = Track::placeEquidistantPoints(localPath,false,-1,m_distanceBetweenPoints);
        } //else One point
      }
    }
  }
  return localPath;
}

float Track::bPID(float dt, float groundSpeed){
  float e = m_aimVel-groundSpeed;
  m_ei += e*dt;
  m_ei=std::max(m_ei,m_axLimitNegative/m_bKi);
  float ed = (e-m_ePrev)/dt;
  m_ePrev=e;
  float accTmp = m_bKp*e+m_bKd*ed+m_bKi*m_ei;
  float accelerationRequest = std::min(std::abs(accTmp),std::abs(m_axLimitNegative));
  if (accTmp<0) {
    accelerationRequest=-accelerationRequest;
  }
  return accelerationRequest;
}

float Track::aPID(float dt, float groundSpeed){
  float e = m_aimVel-groundSpeed;
  if (groundSpeed<=0.0f) {
    m_ei=0.0f;
  }
  else{
    m_ei += e*dt;
  }
  m_ei=std::min(m_ei,m_axLimitPositive/m_aKi);
  float ed = (e-m_ePrev)/dt;
  m_ePrev=e;
  float accTmp = m_aKp*e+m_aKd*ed+m_aKi*m_ei;
  float accelerationRequest = std::min(std::abs(accTmp),m_axLimitPositive);
  if (accTmp<0) {
    accelerationRequest=-accelerationRequest;
  }
  return accelerationRequest;
}

void Track::resetPID(){
  m_ei=0.0f;
  m_ePrev=0.0f;
}
float Track::lowPass(int factor, float lastOutput, float presentReading)
{
  return (factor * lastOutput + presentReading) / (factor+1);
}
