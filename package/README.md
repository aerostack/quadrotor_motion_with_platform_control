# quadrotor_motion_with_platform_control
## behavior_move_vertical_with_platform_control

**Purpose**: The robot moves vertically a given distance. If the distance is positive, the robot moves upward. If the distance is negative, the robot moves downwards.

**Type of behavior:** Goal-based behavior.

**Arguments:** 

| Name    |   Format  |  Example |  
| :-----------| :---------| :--------|
| distance: |number (m)|distance: 1|  

----
## behavior_move_at_speed_with_platform_control

**Purpose:** The robot keeps moving at a constant speed in some direction (forward, backward, left,right). If the speed value is not given a default value is considered. This behavior does not avoid obstacles. 

**Type of behavior:** Recurrent behavior.

| Arguments    |   Format  |  Example |  
| :-----------| :---------| :--------|
| direction |{FORWARD, BACKWARD, LEFT, RIGHT}|direction: FORWARD |          
| speed |number (m/sec)|speed: 0.8|

----
## behavior_rotate_with_platform_control

**Purpose:** The robot rotates left or right a certain number of degrees (angle) on the vertical axis (yaw). The number of degrees can be expressed as an absolute value (argument “angle”) or relative to the robot (argument “relative_angle”).

**Type of behavior:** Goal-based behavior.

| Arguments    |   Format  |  Example |  
| :-----------| :---------| :--------|          
| angle |number (degrees)|angle: 90|
| relative_angle |number (degrees)|relative_angle: 90|

----

# Contributors

**Code maintainer:** Alberto Rodelgo Perales

**Authors:** Alberto Rodelgo Perales