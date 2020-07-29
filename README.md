# ROSK-ROS and FLASK 
The dashbard created for kratos using ROS + FLASK <BR>
Made **USING HTML,CSS,FLASK,ROS,Javascript,JQUERY library,Flot (pure JavaScript plotting library for jQuery),AJAX** for making it a single page app and bootstrap for the template creation.<bR>
The requirements to run the files 
  <ul>
    <li>FLASK
     <li> ROS MELODIC
      <li> <a href="https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-fake-node">GAZEBO</a> (for later purposes)
  </ul>
 
## INSTALLATION
After installing ROS MELODIC go to catkin_ws/src and type 
```
catkin_create_pkg <package name> rospy std_msgs
paste the contents of the src folder into the <package name>/src  and do catkin_make and source devel/setup.bash
go to <package name>/src 
rosrun <package name> move_bot.py 
```
To test the turtlesim robot ,make sure the turtlesim is also running by initiatiing that in the terminal by 
```
rosrun turtlesim turtlesim_node 
then go the http://127.0.0.1:5000/ to view the webpage
``` 

The move_bot.py is for the turtlesim robot ,move_turtle is for the turtlebot3 robot and a gazebo robot should run first for it <br>
the camera.py is for the face detection using opencv<br>
the graph is written using javascript<br>

  
