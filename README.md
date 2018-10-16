# cfsd18-cognition-track
[![Build Status](https://travis-ci.org/cfsd/cfsd18-cognition-track.svg?branch=master)](https://travis-ci.org/cfsd/cfsd18-cognition-track)

## nextContainer
- check if the datatype is GroundSpeedReading, if it is then synchronize the ground speed.
```
if (a_container.dataType() == opendlv::proxy::GroundSpeedReading::ID())
```
- if the datatype is GroundSurfaceProperty, update the time, surface id and frame.
```
if(a_container.dataType() == opendlv::logic::perception::GroundSurfaceProperty::ID())
```
- if the datatype is GroundSurfaceArea, then it gets the coordinates of four vertexes which indicating the quadrangle area on the ground. It also calculates the middle point coordinates and stored in the v vector.
```
if (a_container.dataType() == opendlv::logic::perception::GroundSurfaceArea::ID())
```
```
float x1 = groundSurfaceArea.x1();
float y1 = groundSurfaceArea.y1();
float x2 = groundSurfaceArea.x2();
float y2 = groundSurfaceArea.y2();
float x3 = groundSurfaceArea.x3();
float y3 = groundSurfaceArea.y3();
float x4 = groundSurfaceArea.x4();
float y4 = groundSurfaceArea.y4();
```
```
std::vector<float> v(4);
v[0] = (x1+x2)/2.0f;
v[1] = (y1+y2)/2.0f;
v[2] = (x3+x4)/2.0f;
v[3] = (y3+y4)/2.0f;
```
