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

- removing the points that have negative coordinate values. The variable preSet indicates that the current matrix is still empty.

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

- If it is not the initial matrix (!preSet==TRUE), then resizing the matrix to get rid of the negative path points

```cpp
if (!preSet) {
      Eigen::MatrixXf localPathTmp = localPath.bottomRows(localPath.rows()-count);
      localPath.resize(localPath.rows()-count,2);
      localPath = localPathTmp;
    }
```

- Also check if there is a stop or a one point signal.

- In the matrix, if the the second point coordinates are too small, then there is a stop.

```cpp
if (std::abs(localPath(1,0)) <= 0.0001f && std::abs(localPath(1,1)) <= 0.0001f) {
        Eigen::MatrixXf localPathTmp = localPath.row(0);
        localPath.resize(1,2);
        localPath = localPathTmp;
        STOP = true;
        std::cout << "STOP signal recieved " << std::endl;
      }
```

- If the first point coordinates are too small and there are only two points in the matrix, then it is a one point signal.

```cpp
lse if(std::abs(localPath(0,0)) <= 0.0001f && std::abs(localPath(0,1)) <= 0.0001f && localPath.rows()<3){
        Eigen::MatrixXf localPathTmp = localPath.row(1);
        localPath.resize(1,2);
        localPath = localPathTmp;
        std::cout << "ONE POINT signal recieved " << std::endl;
      }
```

- Calculate the first point

```cpp
Eigen::RowVector2f firstPoint = Track::traceBackToClosestPoint(localPath.row(0), localPath.row(1), Eigen::RowVector2f::Zero(1,2))
```

```cpp
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
```


