# Stretch-find-waterbottle

This is a code that allows a Hello-Robot RE2 robot to move towards an object, while adjusting the robot arm to match the height of the object. This is done using the #1 ArUco tag that is provided by Hello-robot.


## In order for this code to work we need a launch file that contains:
```
  1) Stretch_driver.launch (allows use to control the driver)
  2) rosservice [switch_to_navigation_mode] (allows for us to manually move the robot base movement)
  3) d435i_high_resolution.launch (turns on the RealSense camera)
  4) stretch_aruco.launch (turn on the ability for stretch to read ArUco tags)
  5) aruco_tag_locator.py ( custom code that makes robot move to ArUco tag)
```
For this course, my contribution takes form as the aruco_tag_location.py file.

## Reason for project:
Some individuals with motor impairments have trouble grasping items in their environment, to help the introduction of assistive robots is a possible solution. For this project, I decided to create a way for individuals with motor impairment to be able to get items in their environment with the help of Stretch. This would work by having an individual assist the user by placing an arUco tag on the desired item. Once that is done the user can just hit "run" and the robot will look for that item, move to it, adjust its arm to be aligned with the object, and grasp the item.

Future work: add a way for the robot to return to the original position and drop the item in a location.

## Robot process:
Stretch will start by scanning the environment for the object. Once the object is detected Stretch will position the arm based on the object's position. After that, Stretch will start scanning again (to double-check) and once it detects the object again Stretch will move towards that object.

## Robot video:
![Uploading ROSclass_project_video-1.gif…]()
