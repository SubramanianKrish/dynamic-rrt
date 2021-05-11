# Dynamic RRT

This is a C++ implementation of a Rapidly exploring Random Tree planner for 2D path planning of an unconstrained mobile robot. The environment consists of square and circular obstacles which move in simple pre-defined paths. The planner has local repair capability to modify parts of the tree which are in collision with the obstacles.
<p align="center">
  <img src="https://github.com/SubramanianKrish/dynamic-rrt/blob/main/maze_map.gif?raw=true" alt="Maze Map"/>
</p>
Our implementation is inspired from [this](https://www.youtube.com/watch?v=QLNSkFnBYuM) link.

## Requirements
C ++ 11 compiler  
CMAKE version 3.0  
OpenCV version 3.0  

## Build instructions
In the root directory, exectute the following
```
$ mkdir build && cd build
$ cmake ..
$ make -j2
```
