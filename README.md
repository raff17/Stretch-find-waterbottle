# Stretch-find-waterbottle

This is a code that allows a Hello-Robot RE2 robot to move towards an object, while adjusting the robot arm to match the height of the object. This is done using the #1 ArUco tag that is provided by Hello-robot.

```
In order for this code to work we need a launch file that contains:
  1) Stretch_driver.launch (allows use to control the driver)
  2) rosservice [switch_to_navigation_mode] (allows for us to manually move the robot base movement)
  3) d435i_high_resolution.launch (turns on the RealSense camera)
  4) stretch_aruco.launch (turn on the ability for stretch to read ArUco tags)
  5) aruco_tag_locator.py ( custom code that makes robot move to ArUco tag)
```
