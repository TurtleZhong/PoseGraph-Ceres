###This is a simple example of Pose Graph

###Dependicy
OpenCV 3.2
Ceres
Eigen
...

###How to use
---
mkdir build
cd build
cmake ..
make

----
####Then in the lib folder run:
./pose_graph_ceres

then you will get two txt file: pose_graph.txt and pose graph_before.txt

---

get into the path_plot folder run:
./plot*.sh
then you will get:
![Pose_Graph](image/ceres_KITTI08.png)
