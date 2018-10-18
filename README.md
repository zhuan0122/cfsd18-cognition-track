# cfsd18-cognition-track
[![Build Status](https://travis-ci.org/cfsd/cfsd18-cognition-track.svg?branch=master)](https://travis-ci.org/cfsd/cfsd18-cognition-track)

# cognition-track: main function 
- take the numbers of arguments on the command line and the char pointer which points to another ponter as inputs to test the command line arguments.
-  If the size of the arguments is > 0, then it will excute two conditional functions to convert the information from the comand line to two usigned int32_t constant value: surfaceId and speedId
-  then the OD4Session tool is used for the Interface to a running OpenDaVINCI session
-  call the track function to run the track process with surfaceid and speedid tested condition
- access the datatrigger in GroundSurfaceProperty, GroundSurfaceArea,GroundSpeedReading 

    ## track.cpp
- setUp from commandline, like information for path,steering,velocity control...
- nextContainer function: take a reference: a_container from cluon::data::Envelope to extract different data according to the opendlv::proxy  input id. these information extracted from perception include GroundSpeed,surfaceId,surfaceInframe,GroundSurfaceArea with the localpath point location vetors.



## path 
```
  // make sure the objectID is the current frame id 
  if (m_newId) {
      m_objectId = (objectId!=m_lastObjectId)?(objectId):(-1);
      m_newId=(m_objectId !=-1)?(false):(true);
  } 

 // Unpack location message from perception, mainly for   four corner points in the frame(surfaceArea)

 float x1 = groundSurfaceArea.x1(); 
 float y1 = groundSurfaceArea.y1();
 float x2 = groundSurfaceArea.x2();
 float y2 = groundSurfaceArea.y2();
 float x3 = groundSurfaceArea.x3();
 float y3 = groundSurfaceArea.y3();
 float x4 = groundSurfaceArea.x4();
 float y4 = groundSurfaceArea.y4();

 // get the two midpoints of the surfaceframe, coordinates: (v[0],v[1]), (v[2],v[3])
 std::vector<float> v(4); 
 v[0] = (x1+x2)/2.0f;\n
 v[1] = (y1+y2)/2.0f;\n
 v[2] = (x3+x4)/2.0f;\n
 v[3] = (y3+y4)/2.0f;\n

 // set the local path size and collect points data for the local path

 Eigen::MatrixXf localPath(surfaceFrame.size()*2,2);
 for (std::map<double, std::vector<float> >::iterator it = surfaceFrame.begin();it !=surfaceFrame.end();it++){
      v=it->second;
      localPath(2*I,0)=v[0];
      localPath(2*I,1)=v[1];
      localPath(2*I+1,0)=v[2];
      localPath(2*I+1,1)=v[3];
      I++;
    }
 // remove negative points and Check for stop or "one point" signal for each collecting point and if the positive collecting points is more than 3, it calls
 placeEquidistantPoints function for placing points equidistantly along a sequence of points, taking m_distanceBetweenPoints is equal to 0.5.
 
 localPath = Track::placeEquidistantPoints(localPathCopy,false,-1,m_distanceBetweenPoints);

 // using groundspeed value to get preview distance for the vechile 
  float previewDistance = std::abs(groundSpeedCopy)*m_previewTime;

 // acculate the whole path lenth with all collecting path points coordinates
  float pathLength=localPath.row(0).norm();
  if(localPath.rows()>1){
    for (int i = 0; i < localPath.rows()-1; i++) {
      pathLength+=(localPath.row(i+1)-localPath.row(i)).norm();
    }
  }

 // compare the pathlenth and previewDisatnce, if the preview distace is longer then the previewDistance is equal to pathlenth 
```
## sharp & steering  
 //  the bool variable m_sharp is set to decide which task is excuted, the sharp or the steering. and since it is set false in the track.hpp.
 
 so it will excute driverModelSteering function, which returns a tuple consists headingRequest,distanceToAimPoint variables, they are the values of azimuthAngle and distance of the aimpoint steer in action object. and the steer message will be sent with the m_senderstamp. 
 
 - azimuthAngle 
   ```
   // azimuthAngle is the Angle to aimpoint and it will be limitted by m_wheelAngleLimit, the physical limitations 
  
   headingRequest = atan2f(aimPoint(1),aimPoint(0));
   if (headingRequest>=0) {
      headingRequest = std::min(headingRequest,m_wheelAngleLimit*3.14159265f/180.0f);
   } else {
     headingRequest = std::max(headingRequest,-m_wheelAngleLimit*3.14159265f/180.0f);
   }

 - Aimpoint

   // distanceToAimPoint is calculated by the aimpoint coordinates value, aimpoint is decided by the pathlenth and previewDistance we also mentioned before, the previewDistance is the float value decided by the groundspeed and preciewtime of the vechicle

   if the path lenth is longer than previewDistance and the local path points we colleted is more than 2, then the aimpoint will be placed after the first local path points element. and we will calculate the ampoint through the elements P1, P2. path elements P1, P2 are the last two elements of the local path points, where P2 is the overshoot element since he path lenth is longer than previewDistance. 
   and the aimpoints will be the line interpolation of the line between P1 P2 and the previewDistance line. 

   if  the path lenth is shorter  than previewDistance or the local path points we colleted is less than 2, then Place aimpoint on first path element

   If the path is too short, place aimpoint at the last path element

   Thus the aimpoint is decided by the vehicle and local path 
   ```
## velocity control
 // the driverModelVelocity function returns a float varible accelerationRequest.
  when it is positive, it will be assigned to groundAcceleration object and the acceleration message with senderstamp  also will be sent through OD4Session.
 when the accelerationRequest is negative and then the minus accelerationRequest value will be assigned to groundDeceleration and the deceleration message will be sent with sender stamp.

   // variables involved: 
     groundspeed: normal running speed 
     velocityLimit: the veichle limit velocity
     axLimitPositive/axLimitNagative: max vertical acceleration and deacceleration value
     ayLimit: lateral velocity limit
     speedProfile: a vector storing velocity candidates based on expected lateral acceleration
     curveRadii: a vector storing the return value of curvatureTriCircle function, which is used to calculate the curvature of path for some steps

- !stop& the collecting points numbers in local path are more than 2 
  ```
   // Caluclate curvature of path for setting step, makes sure that step is as big as possible (up to limit)

    while (localPath.rows()-(2*step)<=0 && step > 0){ 
          step--;
        }
        curveRadii = curvatureTriCircle(localPath,step);
      }
   // Segmentize the path and calculate radius of segments: Choose three points with max step and make a triangle with sides A(p1p2),B(p2p3),C(p1p3), also sort side lengths as A >= B && B >= C

    for (int k = 0; k < localPath.rows()-(2*step); k++) {
    
    float A = (localPath.row(k+step)-localPath.row(k)).norm();
    float B = (localPath.row(k+2*step)-localPath.row(k+step)).norm();
    float C = (localPath.row(k+2*step)-localPath.row(k)).norm();
    ...
  
   // then the variable velocity candidates speedProfile could be set for each curveRadii and the ayLimit and the velocity candidates speedProfile is used to check if the brake is needed and also set the brake time.
   // tb is the time to decrease to velocity form the groundspeed to value for  the vechile specific wheel angle limit, if tb is bigger than 0, then the brake is needed.
   // when we need to brake, we need to set the brake time for each velocity candidates(or each path segment)
    
    std::vector<float> distanceToCriticalPoint;
    for (int k=0; k<step; k++){
      s+=(localPath.row(k+1)-localPath.row(k)).norm();
    }
    for (i=0; i<speedProfile.size(); i++){
      s+=(localPath.row(i+step+1)-localPath.row(i+step)).norm();
      distanceToCriticalPoint.push_back(s);
      tb = (speedProfile(i)-groundSpeedCopy)/(axLimitNegative);
      tv = s/groundSpeedCopy; 
      if (tb>0.0f) { 
        tmp = tv-tb; too late
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

  ```
- Stop 
  
  // if the bool variable stop is true then trigger brake to stop 
  ```
    if(STOP){
      if (std::abs(groundSpeedCopy) > 0.01f){
       accelerationRequest = axLimitNegative;
       std::cout << "BREAKING TO STOP " << std::endl;
     }else {accelerationRequest = 0.0f;}
  }
  else{
    accelerationRequest = 0.0f;// not brake no acceleration
  }
  return accelerationRequest;

  ```



















## curvature estimation




    

  

  
  



 


```



