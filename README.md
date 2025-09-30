#ME495 Embedded Systems Homework 1
Author: ${Your Name}
1. Use `ros2 launch turtle_control waypoints.launch.xml` to run the code
2. The `ros2 service call /load turtle_interfaces/srv/Waypoints '{waypoint: [{x: 3.9, y: 5.4, z: 0.0}, {x: 1.4, y: 1.6, z: 0.0}, {x: 2.2, y: 9.4, z: 0.0},{x: 7.2, y: 6.1,z: 0.0},{x: 4.0, y: 2.6, z: 0.0},{x: 8.2, y: 1.5, z: 0}]}'` service loads waypoints for the turtle to follow
3. The `ros2 service call /toggle std_srvs/srv/Empty` starts and stops the turtle.
4. Here is a video of the turtle in action.
https://github.com/user-attachments/assets/39a31867-841f-4cbd-a405-fa3f1a086c58

   ${embed video here, must show up inline in the README.md when rendered on github. Video file itself should be uploaded as a github issue and linked here, not in the repository}`

5. Here is a video when the bag is played.

   ${embed video here, must show up inline in the README.md when rendered on github. Video file itself should be uploaded as a github issue and linked here, not in the repository}`
https://github.com/user-attachments/assets/52fbbf6b-8786-4f99-aef8-4145e92a6a84

