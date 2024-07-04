# ROS

## Dependencies
`ament_cmake`  
`rclcpp`  
`std_msgs`  
`image_transport`  
`OpenCV`  
`cv_bridge`  
`Iconv`  
`VTK`  
`Eigen3`  

## Build 
Flags for OpenCV: `cmake -DWITH_OPENEXR=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x`  
  
From the root of the repository run `colcon build`

## Execute
From the root of the repository run `. install/setup.bash`  
  
Open 3 terminals and run one command for each terminal (the applicant should be the last one):  
Applicant: `ros2 run ROS_PickingPoint applicant`  
Picking Point Handler: `ros2 run ROS_PickingPoint pickingPointHandler`  
Receiver: `ros2 run ROS_PickingPoint receiver`  

## For Arch
Install these dependencies because ros2 doesn't include them by default

`sudo pacman -S fast_float cgns netcdf libharu adios2 liblas pdal boost ospray cli11`
