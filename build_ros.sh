echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake -j 24 -trace .. -DROS_BUILD_TYPE=Release
make -j
