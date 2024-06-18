# ROS

## Build 
From the root of the repository run `colcon build`

## Execute
From the root of the repository run `. install/setup.bash`  
  
Open 3 terminals and run one command for each terminal (the applicant should be the last one):  
Applicant: `ros2 run ROS_PickingPoint applicant`  
Picking Point Handler: `ros2 run ROS_PickingPoint pickingPointHandler`  
Receiver: `ros2 run ROS_PickingPoint receiver`  