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
 //  the bool variable m_sharp is set to decide which task is excuted, the sharo or the steering. and since it is set false, so it will excute driverModelSteering function, which returns a tuple consists headingRequest,distanceToAimPoint values, they are the azimuthAngle and distance of the aimpoint steer in action object. 
 
 - azimuthAngle 
   ```
   //Limit heading request due to physical limitations  and azimuthAngle is the Angle to aimpoint

   headingRequest = atan2f(aimPoint(1),aimPoint(0));
   if (headingRequest>=0) {
      headingRequest = std::min(headingRequest,m_wheelAngleLimit*3.14159265f/180.0f);
   } else {
     headingRequest = std::max(headingRequest,-m_wheelAngleLimit*3.14159265f/180.0f);
   }

 - Aimpoint

   // distanceToAimPoint is calculated by the aimpoint,coordinates value, aimpoint is decided by the pathlenth and previewDistance we also mentioned before, the previewDistance is float decided by the groundspeed and preciewtime of the vechicle

   if the path lenth is longer than previewDistance and the local path points we colleted is more than 2, then the aimpoint will be placed after the first local path points element. path elements P1, P2 are the last two elements of the local path points, where P2 is the overshoot element since he path lenth is longer than previewDistance. 
   
   and the aimpoints will be the line interpolation of the line between P1 P2 and the previewDistance line. 

   if  the path lenth is shorter  than previewDistance or the local path points we colleted is less than 2, then Place aimpoint on first path element

   If the path is too short, place aimpoint at the last path element
   ```
## velocity control
- acceleration 
  ```

  ```
- deacceleration
  ```

  ```



















## curvature estimation




    

  

  
  



 


```



