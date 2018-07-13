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

Track::Track(std::map<std::string, std::string> commandlineArguments, cluon::OD4Session &od4) :
  m_od4(od4),
  m_groundSpeed{0.0f},
  m_groundSpeedMutex{},
  m_lateralAcceleration{0.0f},
  m_lateralAccelerationMutex{},
  m_tick{},
  m_tock{},
  m_tickDt{},
  m_tockDt{},
  m_steerTickDt{},
  m_steerTockDt{},
  m_newClock{true},
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
  m_start{true},
  m_prevHeadingRequest{0.0f},
  m_slamActivated{false},
  m_paramsUpdated{false},
  m_sendMutex()
{
 setUp(commandlineArguments);
 m_tickDt = std::chrono::system_clock::now();
 m_steerTickDt = std::chrono::system_clock::now();
}
Track::~Track()
{
}
void Track::setUp(std::map<std::string, std::string> commandlineArguments)
{
  m_senderStamp=(commandlineArguments["id"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["id"]))) : (m_senderStamp);
  // path
  m_distanceBetweenPoints=(commandlineArguments["distanceBetweenPoints"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["distanceBetweenPoints"]))) : (m_distanceBetweenPoints);
  m_traceBack=(commandlineArguments["useTraceBack"].size() != 0) ? (std::stoi(commandlineArguments["useTraceBack"])==1) : (false);
  // steering
  m_moveOrigin=(commandlineArguments["useMoveOrigin"].size() != 0) ? (std::stoi(commandlineArguments["useMoveOrigin"])==1) : (true);
  m_curveFitPath=(commandlineArguments["useCurveFitPath"].size() != 0) ? (std::stoi(commandlineArguments["useCurveFitPath"])==1) : (true);
  m_previewTime=(commandlineArguments["previewTime"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["previewTime"]))) : (m_previewTime);
  m_minPrevDist=(commandlineArguments["minPrevDist"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["minPrevDist"]))) : (m_minPrevDist);
  m_steerRate=(commandlineArguments["steerRate"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["steerRate"]))) : (m_steerRate);
  m_previewTimeSlam=(commandlineArguments["previewTimeSlam"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["previewTimeSlam"]))) : (m_previewTimeSlam);
  m_minPrevDistSlam=(commandlineArguments["minPrevDistSlam"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["minPrevDistSlam"]))) : (m_minPrevDistSlam);
  m_steerRateSlam=(commandlineArguments["steerRateSlam"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["steerRateSlam"]))) : (m_steerRateSlam);
  // sharp
  m_sharp=(commandlineArguments["useSharp"].size() != 0) ? (std::stoi(commandlineArguments["useSharp"])==1) : (false);
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
  m_axSpeedProfile=(commandlineArguments["axSpeedProfile"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["axSpeedProfile"]))) : (m_axSpeedProfile);
  m_useAyReading=(commandlineArguments["useAyReading"].size() != 0) ? (std::stoi(commandlineArguments["useAyReading"])==1) : (true);
  m_velocityLimit=(commandlineArguments["velocityLimit"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["velocityLimit"]))) : (m_velocityLimit);
  m_mu=(commandlineArguments["mu"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["mu"]))) : (m_mu);
  m_axLimitPositive=(commandlineArguments["axLimitPositive"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["axLimitPositive"]))) : (m_axLimitPositive);
  m_axLimitNegative=(commandlineArguments["axLimitNegative"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["axLimitNegative"]))) : (m_axLimitNegative);
  m_headingErrorDependency=(commandlineArguments["headingErrorDependency"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["headingErrorDependency"]))) : (m_headingErrorDependency);
  m_curveDetectionAngle=(commandlineArguments["curveDetectionAngle"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["curveDetectionAngle"]))) : (m_curveDetectionAngle);
  m_curveDetectionPoints=(commandlineArguments["curveDetectionPoints"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["curveDetectionPoints"]))) : (m_curveDetectionPoints);
  // ....controller
  m_aimVel=(commandlineArguments["startAimVel"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["startAimVel"]))) : (m_aimVel);
  m_keepConstVel=(commandlineArguments["keepConstVel"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["keepConstVel"]))) : (m_keepConstVel);
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
  m_wheelBase=(commandlineArguments["wheelBase"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["wheelBase"]))) : (m_wheelBase);
  m_frontToCog=(commandlineArguments["frontToCog"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["frontToCog"]))) : (m_frontToCog);

  /*//std::cout<<"Track set up with "<<commandlineArguments.size()<<" commandlineArguments: "<<std::endl;
  for (std::map<std::string, std::string >::iterator it = commandlineArguments.begin();it !=commandlineArguments.end();it++){
    //std::cout<<it->first<<" "<<it->second<<std::endl;
  }*/
}

void Track::tearDown()
{
}

void Track::receiveCombinedMessage(std::map<int,opendlv::logic::perception::GroundSurfaceArea> currentFrame, cluon::data::TimeStamp sampleTime){
  m_tick = std::chrono::system_clock::now();
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
  if (a_container.dataType() == opendlv::proxy::GroundSpeedReading::ID()) {
    std::unique_lock<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
    auto groundSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(a_container));
    m_groundSpeed = groundSpeed.groundSpeed();
  }
  else if (a_container.dataType() == opendlv::proxy::AccelerationReading::ID()) {
    std::unique_lock<std::mutex> lockLateralAcceleration(m_lateralAccelerationMutex);
    auto lateralAcceleration = cluon::extractMessage<opendlv::proxy::AccelerationReading>(std::move(a_container));
    m_lateralAcceleration = lateralAcceleration.accelerationY();
  }
  else if (!m_paramsUpdated) {
    if (a_container.dataType() == opendlv::logic::perception::ObjectDirection::ID()) {
      m_slamActivated = true;
    }
  }
}
bool Track::slamParams()
{
  m_keepConstVel = -1.0f;
  m_curveFitPath = false;
  m_steerRate = m_steerRateSlam;
  m_minPrevDist = m_minPrevDistSlam;
  m_previewTime = m_previewTimeSlam;
  return true;
}
void Track::run(Eigen::MatrixXf localPath, cluon::data::TimeStamp sampleTime){
  if (m_slamActivated && !m_paramsUpdated) {
    m_paramsUpdated = slamParams();
  }
  bool noPath = false;
  bool specCase = false;
  int count = 0;
  bool STOP = false;
  float headingRequest;
  float distanceToAimPoint;
  float accelerationRequest;
  float groundSpeedCopy;
  {
    std::unique_lock<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
    groundSpeedCopy = m_groundSpeed;
  }
  if (localPath.rows()<=0 || (localPath.rows()==2 && (std::abs(localPath(0,0))<=0.00001f && std::abs(localPath(1,0))<=0.00001f && std::abs(localPath(0,1))<=0.00001f && std::abs(localPath(1,1))<=0.00001f))){
    noPath = true;
  }
  else{
    // Check for stop or "one point" signal
    if ((std::abs(localPath(localPath.rows()-1,0))<=0.00001f && std::abs(localPath(localPath.rows()-2,0))<=0.00001f && std::abs(localPath(localPath.rows()-1,1))<=0.00001f && std::abs(localPath(localPath.rows()-2,1))<=0.00001f)){
      Eigen::MatrixXf localPathTmp = localPath.topRows(localPath.rows()-2);
      localPath.resize(localPathTmp.rows(),2);
      localPath = localPathTmp;
      STOP = true;
      specCase = true;
    }
    else if(localPath.rows()==2 && (std::abs(localPath(0,0))<=0.00001f && std::abs(localPath(0,1))<=0.00001f)){
      Eigen::MatrixXf localPathTmp = localPath.row(1);
      localPath.resize(1,2);
      localPath = localPathTmp;
      specCase = true;
    }
    if (!specCase) {
      // Order path
      localPath = orderCones(localPath);
      // Remove negative path points
      if (localPath(0,0)<0.0f) {
        while (localPath(count,0)<0.0f){
          count++;
          if (count>localPath.rows()-1) {
            STOP = true;
            noPath = true;
            localPath.resize(1,2);
            localPath <<  1, 0;
            break;
          }
        }
      }
      if (!noPath) {
        if (count>0) {
          Eigen::MatrixXf localPathTmp = localPath.bottomRows(localPath.rows()-count);
          localPath.resize(localPath.rows()-count,2);
          localPath = localPathTmp;
        }
        if (localPath.rows()>1) {
          //Place equidistant points
          Eigen::MatrixXf localPathCopy;
          if (m_traceBack){ //TODO remove function if never used
            Eigen::RowVector2f firstPoint = Track::traceBackToClosestPoint(localPath.row(0), localPath.row(1), Eigen::RowVector2f::Zero(1,2));
            localPathCopy.resize(localPath.rows()+1,2);
            localPathCopy.row(0) = firstPoint;
            localPathCopy.block(1,0,localPath.rows(),2) = localPath;
            localPath.resize(localPathCopy.rows(),2);
          } else{
            localPathCopy = localPath;
          }
          localPath = Track::placeEquidistantPoints(localPathCopy,false,-1,m_distanceBetweenPoints);
        } //else One point
      }
    }

    if (m_sharp) {
      float previewDistance = std::abs(groundSpeedCopy)*m_previewTime;
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
      auto steering = Track::driverModelSteering(localPath, groundSpeedCopy, m_previewTime);
      headingRequest = std::get<0>(steering);
      distanceToAimPoint = std::get<1>(steering);
    }
  }

  if (noPath) {
    headingRequest=m_prevHeadingRequest;
    distanceToAimPoint=1.0f;
  }
  accelerationRequest = Track::driverModelVelocity(localPath, groundSpeedCopy, m_velocityLimit, m_axLimitPositive, m_axLimitNegative, headingRequest, m_headingErrorDependency, m_mu, STOP, noPath);

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
    m_tock = std::chrono::system_clock::now();
    std::chrono::duration<double> dur = m_tock-m_tick;
    m_newClock = true;
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
  /*std::cout << "Here is the matrix M:\n" << M << std::endl;
  std::cout << "Here is the right hand side b:\n" << b << std::endl;
  std::cout << "The least-squares solution is:\n"
      << a << std::endl;
  std::cout<<"localPath:\n "<<matrix<<std::endl;*/
  return a;
}

Eigen::RowVector2f Track::traceBackToClosestPoint(Eigen::RowVector2f p1, Eigen::RowVector2f p2, Eigen::RowVector2f q)
{
   // Input: The coordinates of the first two points. (row vectors)
   //        A reference point q (vehicle location)
   // Output: the point along the line that is closest to the reference point.

   Eigen::RowVector2f v = p1-p2;	// The line to trace
   Eigen::RowVector2f n(1,2);	// The normal vector
   n(0,0) = -v(0,1); n(0,1) = v(0,0);
   //float d = (p1(0,0)*v(0,1)-v(0,0)*p1(0,1))/(n(0,0)*v(0,1)-v(0,0)*n(0,1)); // Shortest distance between [0,0] and the vector
   float d = (v(0,1)*(p1(0,0)-q(0,0))+v(0,0)*(q(0,1)-p1(0,1)))/(n(0,0)*v(0,1)-v(0,0)*n(0,1)); // Shortest distance between q and the vector
   return q+n*d;       // Follow the normal vector for that distance
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

    float ey = std::atan2(localPath(1,1)-localPath(0,1),localPath(1,0)-localPath(0,0));

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


std::tuple<float, float> Track::driverModelSteering(Eigen::MatrixXf localPath, float groundSpeedCopy, float previewTime) {
  float headingRequest;
  float distanceToAimPoint;
  Eigen::Vector2f aimPoint(2);
  bool noPath = false;
  if (m_moveOrigin && localPath.rows()>2) {
    // Move localPath to front wheel axis
    Eigen::MatrixXf foo = Eigen::MatrixXf::Zero(localPath.rows(),2);
    foo.col(0).fill(m_frontToCog);
    localPath = localPath-foo;

    // Remove negative path points
    if (localPath(0,0)<0.0f && localPath.rows()>0) {
      int count = 0;
      while (localPath(count,0)<0.0f){
          count++;
          if (count>localPath.rows()-1) {
            noPath = true;
            break;
          }
      }
      if(!noPath){
        Eigen::MatrixXf localPathTmp = localPath.bottomRows(localPath.rows()-count);
        localPath.resize(localPath.rows()-count,2);
        localPath = localPathTmp;
        if (m_curveFitPath) {
          float sumPoints = localPath.row(0).norm();
          int k=0;
          while (10.0f >= sumPoints && k < localPath.rows()-1) {
            sumPoints += (localPath.row(k+1)-localPath.row(k)).norm();
            k++;
            if (localPath(k+1,0)<localPath(k,0)) {
              break;
            }
          }
          localPath = localPath.topRows(k);
          float xNorm = localPath(0,0);
          float yNorm = localPath(0,1);
          localPath.col(0)=localPath.col(0).array()-xNorm;
          localPath.col(1)=localPath.col(1).array()-yNorm;
          Eigen::VectorXf a = curveFit(localPath);
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
        std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
        cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
        opendlv::body::ActuatorInfo plot;
        plot.x(a(1));
        plot.y(a(2));
        if (a.size()>3) {
          plot.z(a(3));
        }
        else{
          plot.z(0.0f);
        }
        plot.minValue(a(0));
        plot.maxValue(localPath(0,0));
        m_od4.send(plot, sampleTime, 11);
        }
      }
    }
  }
  if (!noPath) {
    // Calculate the distance between vehicle and aimpoint;
    float previewDistance = std::max(std::abs(groundSpeedCopy)*previewTime,m_minPrevDist);
    float sumPoints = localPath.row(0).norm();
    // Sum the distance between all path points until passing previewDistance
    // or reaching end of path
    int k=0;
    while (previewDistance >= sumPoints && k < localPath.rows()-1) {
      sumPoints += (localPath.row(k+1)-localPath.row(k)).norm();
      k++;
    }

    /*k=0;
    sumPoints = localPath(0,0); //TODO: This is a test
    while (previewDistance >= sumPoints && k < localPath.rows()-1) {
      sumPoints += (localPath(k+1,0)-localPath(k,0));
      k++;
    }*/

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
    // Angle to aimpoint
    headingRequest = atan2f(aimPoint(1),aimPoint(0));
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

  m_steerTockDt = std::chrono::system_clock::now();
  std::chrono::duration<float> DT = m_steerTockDt-m_steerTickDt;
  float dt = (DT.count()<1.0f) ? (DT.count()) : (0.1f); // Avoid large DT's to give high control outputs
  if (std::abs(headingRequest-m_prevHeadingRequest)/dt>(m_steerRate*m_PI/180.0)){
    if (headingRequest > m_prevHeadingRequest) {
      headingRequest = dt*m_steerRate*m_PI/180.0f + m_prevHeadingRequest;
    }
    else{
      headingRequest = -dt*m_steerRate*m_PI/180.0f + m_prevHeadingRequest;
    }
  }
  m_prevHeadingRequest=headingRequest;
  m_steerTickDt = std::chrono::system_clock::now();
  return std::make_tuple(headingRequest,distanceToAimPoint);
}

float Track::driverModelVelocity(Eigen::MatrixXf localPath, float groundSpeedCopy, float velocityLimit, float axLimitPositive, float axLimitNegative, float headingRequest, float headingErrorDependency, float mu, bool STOP, bool noPath){
  float accelerationRequest = 0.0f;
  Eigen::VectorXf speedProfile;
  std::vector<float> curveRadii;
  int step = m_step;
  float g = 9.81f;
  if (m_axSpeedProfile<0) {
    m_axSpeedProfile = std::max(axLimitPositive,-axLimitNegative);
  }
  float ayLimit = sqrtf(powf(mu*g,2)-powf(m_axSpeedProfile,2));
  if (std::isnan(ayLimit)) {
    ayLimit = 1.0f;
  }

  m_tockDt = std::chrono::system_clock::now();
  std::chrono::duration<float> DT = m_tockDt-m_tickDt;
  float dt = (DT.count()<1.0f) ? (DT.count()) : (0.1f); // Avoid large DT's to give high control outputs
  if (std::abs(m_ei)>1000.0f) { //Avoid unreasonable integral parts of PID to damage the system
    if (m_ei>0)
      m_ei = 1000.0f;
    else
      m_ei =-1000.0f;
  }

  float e = 0.0f;
  float ed = 0.0f;
  if ((!STOP) && (localPath.rows() > 2) && (m_keepConstVel<0) && (!m_start)){
    m_specCase = false;
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
    speedProfile.resize(curveRadii.size());
    for (uint32_t k = 0; k < curveRadii.size(); k++){
    speedProfile(k) = std::min(sqrtf(ayLimit*curveRadii[k]),velocityLimit);//*(1.0f-headingError*headingErrorDependency);
    }

    Eigen::MatrixXf P;
    int aIdx=-1;
    int curveDetectionPoints = (static_cast<int>(curveRadii.size())<m_curveDetectionPoints) ? (curveRadii.size()-1):(m_curveDetectionPoints);
    for (int k = step; k < localPath.rows()-step-curveDetectionPoints; k++) {
      P=localPath.row(k+curveDetectionPoints)-localPath.row(k);
      float angle = atan2f(P(1),P(0));
      if (std::abs(angle)>m_curveDetectionAngle) {
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
      speedProfile(aIdx)=speedProfile(rIdx);
    }

    float tb;
    float tv;
    float s=0.0f;
    float diff;
    float critDiff=100000.0f;
    float critTb = -0.1f;
    int i;
    int idx = 0;

    for (int k=0; k<step; k++){
      s+=(localPath.row(k+1)-localPath.row(k)).norm();
    }
    for (i=0; i<speedProfile.size(); i++){
      s+=(localPath.row(i+step+1)-localPath.row(i+step)).norm();
      tb = (speedProfile(i)-groundSpeedCopy)/(axLimitNegative);//time to reach velocity if braking max
      tv = s/groundSpeedCopy;
      diff = tv-tb; // when we need to brake, if =0 we need to brake now, if <0 we are braking too late
      if(diff<critDiff && tb > 0){ // save critical brake point
        critDiff=diff;
        critTb = tb;
        idx=i;
      }
    }

    /*---------------State selection---------------*/
    /*BRAKING STATE*/
    if(critDiff<=m_critDiff){ //braking is critical
      if (((groundSpeedCopy-speedProfile(idx))>m_diffToBrakeVel) || m_brakingState) {
        if (!m_brakingState) {
        }
        m_aimVel = speedProfile(idx);
        m_timeToCritVel = critTb;
        m_ei=0.0f;
        m_ePrev=0.0f;
        m_brakingState = true;
        m_rollingState = false;
        m_accelerationState = false;
        m_apexRadius=100000.0f;
      }
      else{ //Enter rolling state instead of braking
        m_brakingState = true;
        m_rollingState = true;
        m_accelerationState = false;
      }
    }
    /*ACCELERATION STATE*/
    if (!m_brakingState){
      if (!m_accelerationState) {
        m_accClock = 0.0f;
        if (aIdx<0) {
          m_aimVel = speedProfile(0)*(1.0f-headingError*headingErrorDependency);
        }
        m_ei=0.0f;
        m_ePrev=0.0f;
      }
      if (aIdx==0) {
        if ((m_minRadius-m_apexRadius)>1.0f) {
          if (m_rollingState) {
          }
          m_aimVel = speedProfile(0)*(1.0f-headingError*headingErrorDependency);
          m_apexRadius=0.0f;
          m_brakingState = false;
          m_rollingState = false;
          m_accelerationState = true;
        }
        else{
          m_brakingState = false;
          m_rollingState = true;
          m_accelerationState = false;
        }
      }
      else{
        m_brakingState = false;
        m_rollingState = false;
        m_accelerationState = true;
      }
    }
    /*ROLLING STATE*/
    if (!m_brakingState && !m_accelerationState) {
      if (!m_rollingState) {
      }
      m_brakingState = false;
      m_rollingState = true;
      m_accelerationState = false;
    }
  /*---------------State execution---------------*/
    if (m_brakingState) {
      e = m_aimVel-groundSpeedCopy;
      m_ei += e*dt;
      ed = (e-m_ePrev)/dt;
      m_ePrev=e;
      if (groundSpeedCopy>m_aimVel) {
        float accTmp = m_bKp*e+m_bKd*ed+m_bKi*m_ei;
        accelerationRequest = std::max(accTmp,axLimitNegative);
      }
      else{
        m_brakingState = false;
        m_rollingState = true;
      }
    }

    if (m_rollingState) {
      if (groundSpeedCopy>1.0f) {
        accelerationRequest = 0.0f;
      }else{
        m_accelerationState = true;
        m_rollingState = false;
      }
    }

    if (m_accelerationState) {
      m_accClock+=DT.count();
      if (m_accClock>(1.0f/m_accFreq)) {
        m_accClock = 0.0f;
        m_aimVel = speedProfile(0);
        m_ei=0.0f;
        m_ePrev=0.0f;
      }

      e = m_aimVel-groundSpeedCopy;
      m_ei += e*dt;
      ed = (e-m_ePrev)/dt;
      m_ePrev=e;
      float accTmp = m_aKp*e+m_aKd*ed+m_aKi*m_ei;
      if (m_aimVel>groundSpeedCopy) {
        accelerationRequest = std::min(accTmp,axLimitPositive);
      }
      else {
        accelerationRequest = 0.0f;
      }
    }
  } //end if(!STOP && (localPath.rows() > 2 ....)
  /*SET SPECIAL CASES*/
  else{
    m_brakingState = false;
    m_rollingState = false;
    m_accelerationState = false;
    if (!m_specCase) {
      m_ei=0.0f;
      m_ePrev=0.0f;
      if (STOP) {
        m_aimVel=0.0f;
      }
      m_specCase=true;
    }
  }
  /*EXECUTE SPECIAL CASES*/
  if (STOP) {
    e = m_aimVel-groundSpeedCopy;
    m_ei += e*dt;
    ed = (e-m_ePrev)/dt;
    m_ePrev=e;
    float accTmp = m_bKp*e+m_bKd*ed+m_bKi*m_ei;
    accelerationRequest = std::max(accTmp,axLimitNegative);
  }
  else if (m_start) {
    e = m_aimVel-groundSpeedCopy;
    m_ei += e*dt;
    ed = (e-m_ePrev)/dt;
    float accTmp = m_sKp*e+m_sKd*ed+m_sKi*m_ei;
    accelerationRequest = std::min(std::abs(accTmp),axLimitPositive);
    if (accTmp<0) {
      accelerationRequest=-accelerationRequest;
    }
    if (groundSpeedCopy > m_aimVel) {
      m_start = false;
    }
  }
  else if (m_keepConstVel>0.0f) {
    e = m_keepConstVel-groundSpeedCopy;
    m_ei += e*dt;
    ed = (e-m_ePrev)/dt;
    float accTmp = m_aKp*e+m_aKd*ed+m_aKi*m_ei;
    accelerationRequest = std::min(std::abs(accTmp),axLimitPositive);
    if (accTmp<0) {
      accelerationRequest=-accelerationRequest;
    }
  }
  else if (noPath || (localPath.rows()<3)) {
    e = m_aimVel-groundSpeedCopy;
    m_ei += e*dt;
    ed = (e-m_ePrev)/dt;
    float accTmp = m_aKp*e+m_aKd*ed+m_aKi*m_ei;
    accelerationRequest = std::min(std::abs(accTmp),axLimitPositive);
    if (accTmp<0) {
      accelerationRequest=-accelerationRequest;
    }
  }
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
  int n = m_polyDeg;
  int pointsPerSegment = m_pointsPerSegment;
  int i,j,segments,N;
  int k=0;
  bool BREAK=false;

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
  Eigen::VectorXf a(n+1);
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

      Eigen::VectorXf X(2*n+1);                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
      for (i=0;i<2*n+1;i++){
        X(i)=0;
        for (j=0;j<N;j++){
          X(i)=X(i)+powf(x(j),i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
        }
      }
      //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
      Eigen::MatrixXf B(n+1,n+2);
      for (i=0;i<=n;i++){
        for (j=0;j<=n;j++){
          B(i,j)=X(i+j);            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
        }
      }
      Eigen::VectorXf Y(n+1);                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
      for (i=0;i<n+1;i++){
        Y(i)=0;
        for (j=0;j<N;j++){
          Y(i)=Y(i)+powf(x(j),i)*y(j);        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
        }
      }
      for (i=0;i<=n;i++){
        B(i,n+1)=Y(i);                //load the values of Y as the last column of B(Normal Matrix but augmented)
      }
      for (i=0;i<n+1;i++){                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k=i+1;k<n+1;k++){
          if (B(i,i)<B(k,i)){
            for (j=0;j<=n+1;j++){
              float temp=B(i,j);
              B(i,j)=B(k,j);
              B(k,j)=temp;
            }
          }
        }
      }
      for (i=0;i<n;i++){            //loop to perform the gauss elimination
        for (k=i+1;k<n+1;k++){
          float t=B(k,i)/B(i,i);
          for (j=0;j<=n+1;j++){
            B(k,j)=B(k,j)-t*B(i,j);    //make the elements below the pivot elements equal to zero or elimnate the variables
          }
        }
      }
      for (i=n;i>=0;i--){                //back-substitution
        a(i)=B(i,n+1);                //make the variable to be calculated equal to the rhs of the last equation
        for (j=0;j<n+1;j++){
          if (j!=i){            //then subtract all the lhs values except the coefficient of the variable whose value is being calculated
            a(i)=a(i)-B(i,j)*a(j);
          }
        }
        a(i)=a(i)/B(i,i);            //now finally divide the rhs by the coefficient of the variable to be calculated
      }

      R.resize(x.size()); // stores curvatures
      for(uint32_t m=0; m<R.size();m++){
        R[m] = 1/std::abs(2*a(2)+6*a(3)*x(m))/powf(1+powf(a(1)+2*a(2)*x(m)+3*a(3)*powf(x(m),2),2),1.5);
        if (R[m]<9.0f) {
          R[m]=9.0f;
        }
      }
      curveRadii.insert(curveRadii.end(), R.begin(), R.end());
    } // end p-loop
  } // end P-loop

  return curveRadii;
} // end curvaturePolyFit
