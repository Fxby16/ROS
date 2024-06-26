# ROS

## Build 
Flags for OpenCV: `cmake -DWITH_OPENEXR=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x`  
  
From the root of the repository run `colcon build`

## Execute
From the root of the repository run `. install/setup.bash`  
  
Open 3 terminals and run one command for each terminal (the applicant should be the last one):  
Applicant: `ros2 run ROS_PickingPoint applicant`  
Picking Point Handler: `ros2 run ROS_PickingPoint pickingPointHandler`  
Receiver: `ros2 run ROS_PickingPoint receiver`  