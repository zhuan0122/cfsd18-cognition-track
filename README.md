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


  ```
  // make sure the objectID is the current frame id 
  if (m_newId) {
      m_objectId = (objectId!=m_lastObjectId)?(objectId):(-1);
      m_newId=(m_objectId !=-1)?(false):(true);
  } 
// Unpack location message from perception, mainly for four corner points in the frame(surfaceArea)
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
v[0] = (x1+x2)/2.0f;
v[1] = (y1+y2)/2.0f;
v[2] = (x3+x4)/2.0f;
v[3] = (y3+y4)/2.0f;

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
  ```



