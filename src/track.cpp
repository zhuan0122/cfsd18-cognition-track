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
#include <thread>
#include "track.hpp"
#include <chrono>

Track::Track(std::map<std::string, std::string> commandlineArguments, cluon::OD4Session &od4) :
  m_od4(od4),
  m_groundSpeed{0.0f},
  m_newFrame{true},
  m_objectId{1},
  m_groundSpeedMutex{},
  m_surfaceMutex{},
  m_surfaceFrame{},
  m_surfaceFrameBuffer{},
  m_nSurfacesInframe{},
  m_surfaceId{},
  m_timeReceived{},
  m_lastObjectId{},
  m_newId{true},
  m_tick{},
  m_tock{},
  m_newClock{true}
{
 setUp(commandlineArguments);
}
Track::~Track()
{
}
void Track::setUp(std::map<std::string, std::string> commandlineArguments)
{
  m_senderStamp=(commandlineArguments["id"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["id"]))) : (m_senderStamp);
  // path
  m_receiveTimeLimit=(commandlineArguments["receiveTimeLimit"].size() != 0) ? (static_cast<double>(std::stod(commandlineArguments["receiveTimeLimit"]))) : (m_receiveTimeLimit); //TODO do we really need to static cast a stof to float?
  m_distanceBetweenPoints=(commandlineArguments["distanceBetweenPoints"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["distanceBetweenPoints"]))) : (m_distanceBetweenPoints);
  m_traceBack=(commandlineArguments["useTraceBack"].size() != 0) ? (std::stoi(commandlineArguments["useTraceBack"])==1) : (false);
  // steering
  m_moveOrigin=(commandlineArguments["useMoveOrigin"].size() != 0) ? (std::stoi(commandlineArguments["useMoveOrigin"])==1) : (true);
  m_previewTime=(commandlineArguments["previewTime"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["previewTime"]))) : (m_previewTime);
  // sharp
  m_sharp=(commandlineArguments["useSharp"].size() != 0) ? (std::stoi(commandlineArguments["useSharp"])==1) : (false);
  m_nSharp=(commandlineArguments["nSharpPreviewPoints"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["nSharpPreviewPoints"]))) : (m_nSharp);
  m_K1=(commandlineArguments["sharpK1"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["sharpK1"]))) : (m_K1);
  m_Ky=(commandlineArguments["sharpKy"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["sharpKy"]))) : (m_Ky);
  m_C=(commandlineArguments["sharpBigC"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["sharpBigC"]))) : (m_C);
  m_c=(commandlineArguments["sharpSmallC"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["sharpSmallC"]))) : (m_c);
  // velocity control
  m_axSpeedProfile=(commandlineArguments["axSpeedProfile"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["axSpeedProfile"]))) : (m_axSpeedProfile);
  m_velocityLimit=(commandlineArguments["velocityLimit"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["velocityLimit"]))) : (m_velocityLimit);
  m_mu=(commandlineArguments["mu"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["mu"]))) : (m_mu);
  m_axLimitPositive=(commandlineArguments["axLimitPositive"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["axLimitPositive"]))) : (m_axLimitPositive);
  m_axLimitNegative=(commandlineArguments["axLimitNegative"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["axLimitNegative"]))) : (m_axLimitNegative);
  m_headingErrorDependency=(commandlineArguments["headingErrorDependency"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["headingErrorDependency"]))) : (m_headingErrorDependency);
  // curvature estimation
  m_polyFit=(commandlineArguments["usePolyFit"].size() != 0) ? (std::stoi(commandlineArguments["usePolyFit"])==1) : (false);
  m_step=(commandlineArguments["curvEstStepsize"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["curvEstStepsize"]))) : (m_step);
  m_polyDeg=(commandlineArguments["polynomialDegree"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["polynomialDegree"]))) : (m_polyDeg);
  m_pointsPerSegment=(commandlineArguments["pointsPerPolySegment"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["pointsPerPolySegment"]))) : (m_pointsPerSegment);
  // vehicle specific
  m_wheelAngleLimit=(commandlineArguments["wheelAngleLimit"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["wheelAngleLimit"]))) : (m_wheelAngleLimit);
  m_wheelBase=(commandlineArguments["wheelBase"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["wheelBase"]))) : (m_wheelBase);
  m_frontToCog=(commandlineArguments["frontToCog"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["frontToCog"]))) : (m_frontToCog);

  std::cout<<"Track set up with "<<commandlineArguments.size()<<" commandlineArguments: "<<std::endl;
  for (std::map<std::string, std::string >::iterator it = commandlineArguments.begin();it !=commandlineArguments.end();it++){
    std::cout<<it->first<<" "<<it->second<<std::endl;
  }
}

void Track::tearDown()
{
}

void Track::nextContainer(cluon::data::Envelope &a_container)
{
  if (a_container.dataType() == opendlv::proxy::GroundSpeedReading::ID()) {
    std::unique_lock<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
    auto groundSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(a_container));
    m_groundSpeed = groundSpeed.groundSpeed();
    //std::cout<<"Track received GroundSpeedReading: "<<groundSpeed.groundSpeed()<<std::endl;
  }

  if(a_container.dataType() == opendlv::logic::perception::GroundSurfaceProperty::ID()){
    if (m_newClock) {
      m_newClock = false;
      m_tick = std::chrono::system_clock::now();
    }
  //    m_lastTimeStamp = a_container.getSampleTimeStamp();
      auto surfaceProperty = cluon::extractMessage<opendlv::logic::perception::GroundSurfaceProperty>(std::move(a_container));
      int surfaceId = surfaceProperty.surfaceId();
      auto nSurfacesInframe = surfaceProperty.property();
      //std::cout<<"Track recieved surface property: "<<" property: "<<nSurfacesInframe<<" frame ID: "<<surfaceId<<std::endl;

      if (m_newFrame) { // If true, a frame has just been sent
        m_newFrame = false;
        m_nSurfacesInframe = std::stoul(nSurfacesInframe);
        m_surfaceId = surfaceId; // Currently not used.
        //std::cout << "SurfaceId: " << surfaceId<<std::endl;
        //std::cout << "nSurfacesInframe: " << nSurfacesInframe<<std::endl;
      }

  }

  if (a_container.dataType() == opendlv::logic::perception::GroundSurfaceArea::ID()) {
    int objectId;
    {
    std::unique_lock<std::mutex> lockSurface(m_surfaceMutex);
      auto groundSurfaceArea = cluon::extractMessage<opendlv::logic::perception::GroundSurfaceArea>(std::move(a_container));
      objectId = groundSurfaceArea.surfaceId();
      cluon::data::TimeStamp containerStamp = a_container.sampleTimeStamp();
      double timeStamp = cluon::time::toMicroseconds(containerStamp); // Save timeStamp for sorting purposes;
      if (m_newId) { // TODO: does it need to be global? can it be initialized in another way?
        m_objectId = (objectId!=m_lastObjectId)?(objectId):(-1); // Update object id if it is not remains from an already run frame
        //std::cout << "newId, m_objectId: " <<m_objectId <<std::endl;
        m_newId=(m_objectId !=-1)?(false):(true); // Set new id to false while collecting current frame id, or keep as true if current id is from an already run frame
      }
      //std::cout << "objectId: " <<objectId <<std::endl;
      //std::cout << "m_objectId: " <<m_objectId <<std::endl;
      //std::cout << "m_lastObjectId: " <<m_lastObjectId <<std::endl;
      float x1 = groundSurfaceArea.x1(); //Unpack message
      float y1 = groundSurfaceArea.y1();
      float x2 = groundSurfaceArea.x2();
      float y2 = groundSurfaceArea.y2();
      float x3 = groundSurfaceArea.x3();
      float y3 = groundSurfaceArea.y3();
      float x4 = groundSurfaceArea.x4();
      float y4 = groundSurfaceArea.y4();
      //std::cout<<"Track recieved surface: "<<" x1: "<<x1<<" y1: "<<y1<<" x2: "<<x2<<" y2: "<<y2<<" x3: "<<x3<<" y3: "<<y3<<" x4: "<<x4<<" y4 "<<y4<<" frame ID: "<<objectId<<" timeStamp: "<<timeStamp<<std::endl;

      std::vector<float> v(4);
      v[0] = (x1+x2)/2.0f;
      v[1] = (y1+y2)/2.0f;
      v[2] = (x3+x4)/2.0f;
      v[3] = (y3+y4)/2.0f;

      if (objectId == m_objectId) {
        //std::cout << "objectId in frame: " <<objectId <<std::endl;
        m_surfaceFrame[timeStamp] = v;
        m_timeReceived = std::chrono::system_clock::now(); //Store time for latest message recieved
        /*std::cout << "Surfaces in frame: " <<m_surfaceFrame.size() <<std::endl;
        for (std::map<double, std::vector<float> >::iterator it = m_surfaceFrame.begin();it !=m_surfaceFrame.end();it++){
          v = it->second;
          for (size_t i = 0; i < 4; i++) {
            std::cout<<v[i]<<"\n";
          }
        }*/
      } else if (objectId != m_lastObjectId){ // If message doesn't belong to current or previous frame.
        //std::cout << "objectId in buffer: " <<objectId <<std::endl;
        m_surfaceFrameBuffer[timeStamp] = v; // Place message content coordinates in buffer
        //std::cout << "Surfaces in buffer: " <<m_surfaceFrameBuffer.size() <<std::endl;
        /*for (std::map<double, std::vector<float> >::iterator it = m_surfaceFrame.begin();it !=m_surfaceFrame.end();it++){
          v = it->second;
          for (size_t i = 0; i < 4; i++) {
            std::cout<<v[i]<<"\n";
          }
        }*/
      }
    }
    auto wait = std::chrono::system_clock::now(); // Time point now
    std::chrono::duration<double> dur = wait-m_timeReceived; // Duration since last message recieved to m_surfaceFrame
    double duration = (m_objectId!=-1)?(dur.count()):(-1.0); // Duration value of type double in seconds OR -1 which prevents running the surface while ignoring messages from an already run frame

    if (duration>m_receiveTimeLimit) { //Only for debug
      std::cout<<"DURATION TIME TRACK EXCEEDED: "<<duration<<std::endl;
      std::cout << "Surfaces to run: " <<m_surfaceFrame.size() <<std::endl;
      /*std::vector<float> v(4);
      for (std::map<double, std::vector<float> >::iterator it = m_surfaceFrame.begin();it !=m_surfaceFrame.end();it++){
        v = it->second;
        for (size_t i = 0; i < 4; i++) {
          std::cout<<v[i]<<"\n";
        }
      }*/
    }

    // Run if frame is full or if we have waited to long for the remaining messages
    if ((m_surfaceFrame.size()==m_nSurfacesInframe || duration>m_receiveTimeLimit)) { //!m_newFrame && objectId==m_surfaceId &&
      //std::cout<<"Run condition OK "<<"\n";
      //std::cout << "duration: " <<duration <<std::endl;
      /*std::vector<float> v;
      std::cout<<"Running surface: "<<std::endl;
      for (std::map<double, std::vector<float> >::iterator it = m_surfaceFrame.begin();it !=m_surfaceFrame.end();it++){
        v=it->second;
        std::cout<<v[0]<<" "<<v[1]<<" "<<v[2]<<" "<<v[3]<<std::endl;
      }*/
      std::thread surfaceCollector(&Track::collectAndRun, this); // Run the surface in a thread
      surfaceCollector.detach();
    }
  }
}


void Track::collectAndRun(){
  //std::cout<<"enter collectAndRun"<<"\n";

  std::map< double, std::vector<float> > surfaceFrame;
  {
  std::unique_lock<std::mutex> lockSurface(m_surfaceMutex);
    m_newFrame = true; // Allow for new surface property
    surfaceFrame = m_surfaceFrame; // Copy frame
    m_surfaceFrame = m_surfaceFrameBuffer; // Add buffer to new frame
    m_surfaceFrameBuffer.clear(); // Clear buffer
    m_lastObjectId = m_objectId; // Update last object id to ignore late messages
    m_newId = true; // Allow for new messages to be placed in m_surfaceFrame
    //std::cout << "Cleared buffer " <<std::endl;
  }


  std::vector<float> v;
  Eigen::MatrixXf localPath(surfaceFrame.size()*2,2);
  //std::cout<<"localPath.rows(): "<<localPath.rows()<<"\n";
  {
    std::unique_lock<std::mutex> lockSurface(m_surfaceMutex);
    int I=0;
    for (std::map<double, std::vector<float> >::iterator it = surfaceFrame.begin();it !=surfaceFrame.end();it++){
      v=it->second;
      localPath(2*I,0)=v[0];
      localPath(2*I,1)=v[1];
      localPath(2*I+1,0)=v[2];
      localPath(2*I+1,1)=v[3];
      I++;
    }
  }
  // Remove negative path points
  bool preSet = false;
  bool STOP = false;
  if (localPath(0,0)<0.0f && localPath.rows()>0) {
    int count = 0;
    while (localPath(count,0)<0.0f){
        count++;
        if (count>localPath.rows()-1) {
          STOP = true;
          preSet = true;
          localPath.resize(1,2);
          localPath <<  1, 0;
          break;
        }
    }
    if (!preSet) {
      Eigen::MatrixXf localPathTmp = localPath.bottomRows(localPath.rows()-count);
      localPath.resize(localPath.rows()-count,2);
      localPath = localPathTmp;
    }
  }
  if (!preSet) {
    // Check for stop or "one point" signal
    if(localPath.rows() > 1){
      if (std::abs(localPath(1,0)) <= 0.0001f && std::abs(localPath(1,1)) <= 0.0001f) {
        Eigen::MatrixXf localPathTmp = localPath.row(0);
        localPath.resize(1,2);
        localPath = localPathTmp;
        STOP = true;
        std::cout << "STOP signal recieved " << std::endl;
      }
      else if(std::abs(localPath(0,0)) <= 0.0001f && std::abs(localPath(0,1)) <= 0.0001f && localPath.rows()<3){
        Eigen::MatrixXf localPathTmp = localPath.row(1);
        localPath.resize(1,2);
        localPath = localPathTmp;
        std::cout << "ONE POINT signal recieved " << std::endl;
      }
      else { //Place equidistant points

        Eigen::MatrixXf localPathCopy;
        if (m_traceBack){
          Eigen::RowVector2f firstPoint = Track::traceBackToClosestPoint(localPath.row(0), localPath.row(1), Eigen::RowVector2f::Zero(1,2));
          localPathCopy.resize(localPath.rows()+1,2);
          localPathCopy.row(0) = firstPoint;
          localPathCopy.block(1,0,localPath.rows(),2) = localPath;
          localPath.resize(localPathCopy.rows(),2);
        } else{
          localPathCopy = localPath;
        }
        localPath = Track::placeEquidistantPoints(localPathCopy,false,-1,m_distanceBetweenPoints);
      }
    } //else Only one point remaining
  }
  //std::cout<<"localPath: "<<localPath<<"\n";
  float groundSpeedCopy;
  {
    std::unique_lock<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
    groundSpeedCopy = m_groundSpeed;
  }
  float previewDistance = std::abs(groundSpeedCopy)*m_previewTime;
  float pathLength=localPath.row(0).norm();
  if(localPath.rows()>1){
    for (int i = 0; i < localPath.rows()-1; i++) {
      pathLength+=(localPath.row(i+1)-localPath.row(i)).norm();
    }
  }
  if (previewDistance>pathLength) {
    //std::cout<<"previewDistance "<< previewDistance <<" is longer than pathLength "<<pathLength<<std::endl;
    previewDistance = pathLength;
  }

  float headingRequest;
  float distanceToAimPoint;
  if (m_sharp) {
    headingRequest = Track::driverModelSharp(localPath, previewDistance);
    distanceToAimPoint = 3.0f;
  }
  else{
    auto steering = Track::driverModelSteering(localPath, groundSpeedCopy, m_previewTime);
    headingRequest = std::get<0>(steering);
    distanceToAimPoint = std::get<1>(steering);
  }
  float accelerationRequest = Track::driverModelVelocity(localPath, groundSpeedCopy, m_velocityLimit, m_axLimitPositive, m_axLimitNegative, headingRequest, m_headingErrorDependency, m_mu, STOP);

//std::cout << "Sending headingRequest: " << headingRequest << std::endl;
std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
  opendlv::logic::action::AimPoint steer;
  steer.azimuthAngle(headingRequest);
  steer.distance(distanceToAimPoint);
  m_od4.send(steer, sampleTime, m_senderStamp);

  if (accelerationRequest >= 0.0f) {
//std::cout << "Sending accelerationRequest: " << accelerationRequest << std::endl;
    opendlv::proxy::GroundAccelerationRequest acc;
    acc.groundAcceleration(accelerationRequest);
    m_od4.send(acc, sampleTime, m_senderStamp);
  }
  else if(accelerationRequest < 0.0f){
//std::cout << "Sending decelerationRequest: " << accelerationRequest << std::endl;
    opendlv::proxy::GroundDecelerationRequest dec;
    dec.groundDeceleration(-accelerationRequest);
    m_od4.send(dec, sampleTime, m_senderStamp);
  }
  m_tock = std::chrono::system_clock::now();
  std::chrono::duration<double> dur = m_tock-m_tick;
  m_newClock = true;
  std::cout<<"Track Module Time: "<<dur.count()<<std::endl;
  //std::cout<<"Track send: "<<" headingRequest: "<<headingRequest<<" accelerationRequest: "<<accelerationRequest<<" sampleTime: "<<cluon::time::toMicroseconds(sampleTime)<<std::endl;
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

float Track::driverModelSharp(Eigen::MatrixXf localPath, float previewDistance){
  float headingRequest;
  int n = m_nSharp;
  if (localPath.rows()>2) {
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
          //std::cout<<"index: "<<idx<<std::endl;
          //std::cout<<"localPath(idx): "<<localPath(idx)<<std::endl;
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
      //std::cout<<"K1e1 "<<K1*error[0]<<std::endl;
      //std::cout<<"Kyey "<<Ky*ey<<std::endl;
    float errorSum = 0.0f;
    float j=0.0f;
    for (int i = 1; i < n; i++) {
      errorSum += error[i]*1.0f/expf(j)*m_C;
      //std::cout<<"e "<< i <<": "<<error[i]<<std::endl;
      //std::cout<<"K "<<i<<": "<<1.0f/expf(j)*C<<std::endl;
      //std::cout<<"K[i]e[i] i= "<<i<<": "<<error[i]*1.0f/expf(j)*C<<std::endl;
      j+=m_c;
    }

    headingRequest = m_Ky*ey + m_K1*error[1] + errorSum;
    if (headingRequest>=0) {
      headingRequest = std::min(headingRequest,m_wheelAngleLimit*3.14159265f/180.0f);
    } else {
      headingRequest = std::max(headingRequest,-m_wheelAngleLimit*3.14159265f/180.0f);
    }
  }
  else{
    headingRequest = 0.0f;
  }
  return headingRequest;
}


std::tuple<float, float> Track::driverModelSteering(Eigen::MatrixXf localPath, float groundSpeedCopy, float previewTime) {
  float headingRequest;
  Eigen::Vector2f aimPoint(2);
  bool preSet = false;
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
            aimPoint << 1,
                        0;
            preSet = true;
            break;
          }
      }
      if(!preSet){
        Eigen::MatrixXf localPathTmp = localPath.bottomRows(localPath.rows()-count);
        localPath.resize(localPath.rows()-count,2);
        localPath = localPathTmp;
      }
    }
  }
  if (!preSet) {
    // Calculate the distance between vehicle and aimpoint;
    float previewDistance = std::abs(groundSpeedCopy)*previewTime;
    //std::cout << "previewDistance: "<<previewDistance<<"\n";
    float sumPoints = localPath.row(0).norm();
    // Sum the distance between all path points until passing previewDistance
    // or reaching end of path
    int k=0;
    while (previewDistance >= sumPoints && k < localPath.rows()-1) {
      sumPoints += (localPath.row(k+1)-localPath.row(k)).norm();
      k++;
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
         //interpolation
        /*distanceP1P2 = localPath.row(0).norm(); // Distance is equal to the distance to the first point;
        overshoot = sumPoints - previewDistance;
        distanceP1AimPoint = distanceP1P2 - overshoot;
        aimPoint = localPath.row(0)*(distanceP1AimPoint/distanceP1P2);*/
        aimPoint = localPath.row(0);
      }
    }
    // If the path is too short, place aimpoint at the last path element
    else {
      aimPoint = localPath.row(localPath.rows()-1);
      std::cout << "aimpoint placed on last path element" << '\n';
    }
  }
  // Angle to aimpoint
  headingRequest = atan2f(aimPoint(1),aimPoint(0));
  // Limit heading request due to physical limitations
  if (headingRequest>=0) {
    headingRequest = std::min(headingRequest,m_wheelAngleLimit*3.14159265f/180.0f);
  } else {
    headingRequest = std::max(headingRequest,-m_wheelAngleLimit*3.14159265f/180.0f);
  }

  float distanceToAimPoint=aimPoint.norm();

  return std::make_tuple(headingRequest,distanceToAimPoint);
}

float Track::driverModelVelocity(Eigen::MatrixXf localPath, float groundSpeedCopy, float velocityLimit, float axLimitPositive, float axLimitNegative, float headingRequest, float headingErrorDependency, float mu, bool STOP){
  float accelerationRequest;
  Eigen::VectorXf speedProfile;
  std::vector<float> curveRadii;
  int step = m_step;
  float g = 9.81f;
  if (m_axSpeedProfile<0) {
    m_axSpeedProfile = std::max(axLimitPositive,-axLimitNegative);
  }
  float ayLimit = sqrtf(powf(mu*g,2)-powf(m_axSpeedProfile,2));
  if (std::isnan(ayLimit)) {
    std::cout<<"ayLimit is NAN, axLimit set too High"<<std::endl;
    ayLimit = 1.0f;
    std::cout<<"ayLimit set to: "<<ayLimit<<std::endl;
  }
  if ((!STOP) && (localPath.rows() > 2)){
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

    /*int index = floor(previewDistance/m_distanceBetweenPoints); //This section calculates the angle to the last path point
    if (index>localPath.rows()-1) {
      index = localPath.rows()-1;
      std::cout<<"INDEX TOO FAR (it was possible)"<<std::endl;
    }
    float previewAngle = std::atan2(localPath(index,1),localPath(index,0));
    if (previewAngle>=0) {
      previewAngle = std::min(previewAngle,m_wheelAngleLimit*3.14159265f/180.0f);
    } else {
      previewAngle = std::max(previewAngle,-m_wheelAngleLimit*3.14159265f/180.0f);
    }*/
    float headingError = std::abs(headingRequest)/(m_wheelAngleLimit*3.14159265f/180.0f);
    //std::cout<<"headingError: "<<headingError<<std::endl;
    //std::cout<<"headinError scaling: "<<1.0f-headingError*headingErrorDependency<<std::endl;

    // Set velocity candidate based on expected lateral acceleration
    speedProfile.resize(curveRadii.size());
    for (uint32_t k = 0; k < curveRadii.size(); k++){
    speedProfile(k) = std::min(sqrtf(ayLimit*curveRadii[k]),velocityLimit)*(1.0f-headingError*headingErrorDependency);
    }
    /*---------------------------------------------------------------------------*/
    //std::cout<<"SpeedProfile: "<<speedProfile.transpose()<<std::endl;
    /*for (uint32_t i = 0; i < curveRadii.size(); ++i)
    {
      std::cout<<curveRadii[i]<<std::endl;
    }*/


    //float brakeMetersBefore = 4;
    //int K = round(brakeMetersBefore/m_distanceBetweenPoints);
    float tb;
    float tv;
    float ta = 5.0;
    float s=0;
    float tmp;
    float brakeTime=100000.0f;
    float accPointMin=100000.0f;
    //float rollResistance = -0.2f;
    int i;
    int idx;
    int accIdx = 0;
    std::vector<float> distanceToCriticalPoint;
    for (int k=0; k<step; k++){
      s+=(localPath.row(k+1)-localPath.row(k)).norm();
    }
    for (i=0; i<speedProfile.size(); i++){
      s+=(localPath.row(i+step+1)-localPath.row(i+step)).norm();
      distanceToCriticalPoint.push_back(s);
      tb = (speedProfile(i)-groundSpeedCopy)/(axLimitNegative);//time to reach velocity
      tv = s/groundSpeedCopy;
      if (tb>0.0f) { //braking is needed to reach this velocity
        tmp = tv-tb; // when we need to brake, if =0 we need to brake now, if <0 we are braking too late
        if(tmp<brakeTime){
          brakeTime=tmp;
          idx=i;
        }
      }
      else {
        if (speedProfile(i)<accPointMin) {
          accPointMin = speedProfile(i);
          accIdx = i;
          ta = tv;
        }
      }
    }
    float ay = powf(groundSpeedCopy,2)/(m_wheelBase/std::tan(std::abs(headingRequest)));
    if(brakeTime<=0.0f){ //braking is critical
      /*if (brakeTime<0.0f) {
        std::cout<<"braking too late, brakeTime: "<<brakeTime<<std::endl;
      }*/
      accelerationRequest = axLimitNegative;
      //std::cout<<"brake max: "<<accelerationRequest<<std::endl;
      /*if (sqrtf(powf(ay,2)+powf(accelerationRequest,2)) >= g*mu) {
        accelerationRequest = -sqrtf(powf(g*mu,2)-powf(ay,2))*0.9f;
        std::cout<<"accreq limited: "<<accelerationRequest<<std::endl;
      }
      if (std::isnan(accelerationRequest)) {
        std::cout<<"accelerationRequest 1 is NaN, ay = "<<ay<<std::endl;
        accelerationRequest = 0.0f;
      }*/
    }
    else if (brakeTime>0.0f && brakeTime<=0.3f){//braking is critical soon, brake prematurely
      /*if (idx-K>=0 && curveRadii[idx]<10.0f) {
        accelerationRequest = std::max((speedProfile(idx)-groundSpeedCopy)/(2*distanceToCriticalPoint[idx-K]),axLimitNegative);
      }else{*/
        accelerationRequest = std::max((speedProfile(idx)-groundSpeedCopy)/(2*distanceToCriticalPoint[idx]),axLimitNegative); //Uncomment if other than max braking should be used
        accelerationRequest = axLimitNegative;
        //std::cout<<"brake prematurely: "<<accelerationRequest<<std::endl;
      //}
      /*if (sqrtf(powf(ay,2)+powf(accelerationRequest,2)) >= g*mu) {
        accelerationRequest = -sqrtf(powf(g*mu,2)-powf(ay,2))*0.9f;
        std::cout<<"accreq limited: "<<accelerationRequest<<std::endl;
        if (std::isnan(accelerationRequest)) {
          std::cout<<"accelerationRequest 2 is NaN, ay = "<<ay<<std::endl;
          accelerationRequest = 0.0f;
        }
      }*/
      /*if (accelerationRequest>rollResistance*g) {
        accelerationRequest = 0.0f;
        std::cout<<"roll resistance is enough"<<std::endl;
      }*/
    }
    else if(brakeTime<= 0.5f){ //if we need to brake within this time, don't give acceleration
      accelerationRequest = 0.0f;
      //std::cout<<"no need to accelerate, braking soon: "<<accelerationRequest<<std::endl;
    }
    else if((speedProfile(accIdx)-groundSpeedCopy) < 0.5f && ta<-0.5f){ //UNUSED TODO: check if really unused
      accelerationRequest = 0.0f;
      //std::cout<<"no need to accelerate: "<<accelerationRequest<<std::endl;
    }
    else{
      accelerationRequest = axLimitPositive;
      //std::cout<<"accelerate max: "<<accelerationRequest<<std::endl;
      if (sqrtf(powf(ay,2)+powf(accelerationRequest,2)) >= g*mu) {
        accelerationRequest = sqrtf(powf(g*mu,2)-powf(ay,2))*0.9f;
        //std::cout<<"accreq limited: "<<accelerationRequest<<std::endl;
        if (std::isnan(accelerationRequest)) {
          //std::cout<<"accelerationRequest 3 is NaN, ay = "<<ay<<std::endl;
          accelerationRequest = 0.0f;
        }
      }

    }

    /*---------------------------------------------------------------------------*/
    /*// Back propagate the whole path and lower velocities if deceleration cannot
    // be achieved. //TODO not pow g*mu
    for (int k = speedProfile.size()-1; k > 0 ; k-=1) {
      // Distance between considered path points
      float pointDistance = (localPath.row(k+step)-localPath.row(k+step-1)).norm();
      // Requiered acceleration to achieve velocity of following point from previous point
      float ax = (speedProfile(k)-speedProfile(k-1))/(2.0f*pointDistance);
      // Lateral acceleration at k
      float ay = powf(speedProfile(k),2)/curveRadii[k];
      // If deceleration is needed
      if (ax < 0.0f) {
        // If deceleration is too high for point k, or higher than vehicle specific limit
        if (sqrtf(powf(ay,2)+powf(ax,2)) >= powf(g*mu,2) || ax < axLimitNegative) {
          ax = std::max((-sqrtf(powf(g*mu,2)-powf(powf(speedProfile(k),2)/curveRadii[k],2)))*0.9f,axLimitNegative); //0.9 is a safetyfactor since ax must be less than rhs
          speedProfile(k-1) = sqrtf(powf(speedProfile(k),2)-2.0f*ax*pointDistance);
        }
      }
    }
    std::cout << "speedProfile = " << speedProfile.transpose() << "\n";
    // Choose velocity to achieve
    // Limit it dependent on the path length and heading request (heading error)
    if(m_previewTime*groundSpeedCopy>distanceToAimPoint*1000000.0f){
      speedProfile(0)=distanceToAimPoint/m_previewTime;
    }
    float pathLength = 0;
    for (int k=0; k<localPath.rows()-1; k++) {
      pathLength += (localPath.row(k+1)-localPath.row(k)).norm();
    }
  std::cout << "pathLength: " <<pathLength<< '\n';
    if(m_previewTime*groundSpeedCopy>pathLength){
      speedProfile(0)=distanceToAimPoint/m_previewTime;
    }
    float desiredVelocity = speedProfile(0)/(1.0f + headingErrorDependency*std::abs(headingRequest));
    // Calculate distance to desired velocity
    float distanceToAimVelocity = (localPath.row(step)).norm(); //TODO localPath.row(step) = 0 if spec case one point....
    // Transform into acceleration
    accelerationRequest = (desiredVelocity-groundSpeedCopy)/(2.0f*distanceToAimVelocity); // TODO use ax directly?
    // Limit acceleration request for positive acceleration
    float const wheelBase = getKeyValueConfiguration().getValue<float>("logic-cfsd18-cognition-track.wheelBase");
    if (accelerationRequest > 0.0f && (sqrtf(powf(powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))),2)+powf(accelerationRequest,2)) >= powf(g*mu,2) || accelerationRequest > axLimitPositive)) {
      std::cout << "accelerationRequest pos limited: " << accelerationRequest << std::endl;
      if (powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))) < g*mu){
        accelerationRequest = std::min((sqrtf(powf(g*mu,2)-powf(powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))),2)))*0.9f,axLimitPositive); //0.9 is a safetyfactor since ax must be less than rhs
      }
      else{
        std::cout<<"We are going to fast, just Roll with it"<<"\n";
        accelerationRequest = 0.0f;
      }
    }
    // Limit acceleration request for negative acceleration TODO: use groundSpeed instead of speedProfile(0)?? Fails if driving too fast
    if (accelerationRequest < 0.0f && (sqrtf(powf(powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))),2)+powf(accelerationRequest,2)) >= powf(g*mu,2) || accelerationRequest < axLimitNegative)) {
      std::cout << "accelerationRequest neg limited: " << accelerationRequest << std::endl;
      if (powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))) < g*mu){
      accelerationRequest = std::max((-sqrtf(powf(g*mu,2)-powf(powf(groundSpeedCopy,2)/(wheelBase/std::tan(std::abs(headingRequest))),2)))*0.9f,axLimitNegative); //0.9 is a safetyfactor since ax must be less than rhs
      }
      else{
        std::cout<<"Fuck it, we are already sliding, BREAK HARD"<<"\n";
        accelerationRequest = axLimitNegative;
      }
    }*/
    /*---------------------------------------------------------------------------*/
  } //end if(!STOP && (localPath.rows() > 2)
  else if(STOP){
    if (std::abs(groundSpeedCopy) > 0.01f){
      accelerationRequest = axLimitNegative;
      std::cout << "BREAKING TO STOP " << std::endl;
    } else {accelerationRequest = 0.0f;}
  }
  else{
    accelerationRequest = 0.0f;
  }
  //std::cout << "groundSpeedCopy: " << groundSpeedCopy << std::endl;
  return accelerationRequest;
}

std::vector<float> Track::curvatureTriCircle(Eigen::MatrixXf localPath, int step){
  // Segmentize the path and calculate radius of segments
  // - First radius is calculated at 2nd path point
  // - Last radius is calculated at 2nd to last path point
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
  /*std::cout<<"curveRadii: "<<" ";
  for (size_t i = 0; i < curveRadii.size(); i++) {
    std::cout<<curveRadii[i]<<" ";
  }
  std::cout<<"\n";*/

  return curveRadii;
}

std::vector<float> Track::curvaturePolyFit(Eigen::MatrixXf localPath){ // TODO: double check coordinate system, maybe switch x/y
  auto startPoly = std::chrono::system_clock::now();
// TODO: Fix commandlineArguments for all configuration stuff
  int n = m_polyDeg;
  int pointsPerSegment = m_pointsPerSegment;
  int i,j,segments,N;
  int k=0;
  uint32_t l=0;
  Eigen::VectorXf dividedPathX(localPath.rows()); // TODO: This is now maximum possible size, which in some cases is unneccesary
  Eigen::VectorXf dividedPathY(localPath.rows());
  std::vector<Eigen::VectorXf> dividedPathsX;
  std::vector<Eigen::VectorXf> dividedPathsY;
      //curveRadii.insert(curveRadii.end(), R.begin(), R.end() );
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
        }
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
    segments = pathx.size()/N; //number of segments

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
      x = pathx.segment(segmentBegin,segmentLength).array()-x(0);
      y = pathy.segment(segmentBegin,segmentLength).array()-y(0);

      Eigen::VectorXf X(2*n+1);                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
      for (i=0;i<2*n+1;i++){
        X(i)=0;
        for (j=0;j<N;j++)
            X(i)=X(i)+powf(x(j),i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
      }
      //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
      Eigen::MatrixXf B(n+1,n+2);
      for (i=0;i<=n;i++)
        for (j=0;j<=n;j++)
            B(i,j)=X(i+j);            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
      Eigen::VectorXf Y(n+1);                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
      for (i=0;i<n+1;i++){
        Y(i)=0;
        for (j=0;j<N;j++)
        Y(i)=Y(i)+powf(x(j),i)*y(j);        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
      }
      for (i=0;i<=n;i++)
        B(i,n+1)=Y(i);                //load the values of Y as the last column of B(Normal Matrix but augmented)

      for (i=0;i<n+1;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k=i+1;k<n+1;k++)
            if (B(i,i)<B(k,i))
                for (j=0;j<=n+1;j++){
                    float temp=B(i,j);
                    B(i,j)=B(k,j);
                    B(k,j)=temp;
                }
      for (i=0;i<n;i++)            //loop to perform the gauss elimination
        for (k=i+1;k<n+1;k++){
                float t=B(k,i)/B(i,i);
                for (j=0;j<=n+1;j++)
                    B(k,j)=B(k,j)-t*B(i,j);    //make the elements below the pivot elements equal to zero or elimnate the variables
            }
      for (i=n;i>=0;i--){                //back-substitution
        a(i)=B(i,n+1);                //make the variable to be calculated equal to the rhs of the last equation
        for (j=0;j<n+1;j++)
            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value is being calculated
                a(i)=a(i)-B(i,j)*a(j);
        a(i)=a(i)/B(i,i);            //now finally divide the rhs by the coefficient of the variable to be calculated
      }

      R.resize(x.size()); // stores curvatures
      for(uint32_t m=0; m<R.size();m++){
        R[m] = 1/std::abs(2*a(2)+6*a(3)*x(m))/powf(1+powf(a(1)+2*a(2)*x(m)+3*a(3)*powf(x(m),2),2),1.5);
      }
      curveRadii.insert(curveRadii.end(), R.begin(), R.end());
    } // end p-loop
  } // end P-loop
  auto stopPoly = std::chrono::system_clock::now();
  auto timePoly = std::chrono::duration_cast<std::chrono::microseconds>(stopPoly - startPoly);
  std::cout << "Polyfit Time:" << timePoly.count() << std::endl;
  std::cout<<"curveRadii: "<<" ";
  for (size_t m = 0; m < curveRadii.size(); m++) {
    std::cout<<curveRadii[m]<<" ";
  }
  std::cout<<"\n";
  return curveRadii;
} // end curvaturePolyFit
