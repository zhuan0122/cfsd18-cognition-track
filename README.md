# cfsd18-cognition-track
[![Build Status](https://travis-ci.org/cfsd/cfsd18-cognition-track.svg?branch=master)](https://travis-ci.org/cfsd/cfsd18-cognition-track)

## nextContainer

- check if the datatype is GroundSpeedReading, if it is then synchronize the ground speed.

```cpp
if (a_container.dataType() == opendlv::proxy::GroundSpeedReading::ID())
```

- if the datatype is GroundSurfaceProperty, update the time, surface id and frame.

```cpp
if(a_container.dataType() == opendlv::logic::perception::GroundSurfaceProperty::ID())
```

- if the datatype is GroundSurfaceArea, then it gets the coordinates of four vertexes which indicating the quadrangle area on the ground. It also calculates the middle point coordinates and stored in the v vector.

```cpp
if (a_container.dataType() == opendlv::logic::perception::GroundSurfaceArea::ID())
```

```cpp
float x1 = groundSurfaceArea.x1();
float y1 = groundSurfaceArea.y1();
float x2 = groundSurfaceArea.x2();
float y2 = groundSurfaceArea.y2();
float x3 = groundSurfaceArea.x3();
float y3 = groundSurfaceArea.y3();
float x4 = groundSurfaceArea.x4();
float y4 = groundSurfaceArea.y4();
```

```cpp
std::vector<float> v(4);
v[0] = (x1+x2)/2.0f;
v[1] = (y1+y2)/2.0f;
v[2] = (x3+x4)/2.0f;
v[3] = (y3+y4)/2.0f;
```

- Also the duration of receiving the data is checked here (checking if the duration exceed the receive time limit).

```cpp
// Run if frame is full or if we have waited to long for the remaining messages
if ((m_surfaceFrame.size()==m_nSurfacesInframe || duration>m_receiveTimeLimit))
```

- To run the surface we start a thread which is handled by the function Track::collectAndRun

```cpp
std::thread surfaceCollector(&Track::collectAndRun, this); // Run the surface in a thread
surfaceCollector.detach();
```

## collectAndRun

- There are multiple frame coordinates stored in the matrix called localpath, we update the matrix using the vector v that indicates the coordinates of the middle points.

- removing the points that have negative coordinate values.

```cpp
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
```

