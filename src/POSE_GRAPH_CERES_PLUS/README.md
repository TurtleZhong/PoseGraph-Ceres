# This is a simple example of Pose Graph

# 1.Description
wait to add..


# 2. Prerequisites
We have tested the library in **16.04**, but it should be easy to compile in other platforms.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## Ceres
Download and install instructions can be found at: http://www.ceres-solver.org/installation.html.

## Sophus [++Sophus has been removed!!!++]
git@github.com:strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build & cd build
cmake ..
sudo make install

# 3. Folder format
**wait to add**

# 4. How to build
mkdir build
cd build
cmake ..
make

# 5. How to run
####Then in the bin folder run:
./pose_graph_ceres

then you will get result in result/

---

# 6. Show the results

get into the result/PlotShow folder run:
./plot*.sh
then you will get:
![Pose_Graph](result/PlotShow/pose_graph.png)
and trajectory
![x_z](result/PlotShow/KITTI00_x_z.png)
# Good Luck!
