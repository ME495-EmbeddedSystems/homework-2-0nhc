# ME495 Embedded Systems Homework 2
Author: Zhengxiao Han

`turtle_brick`: This package includes an arena node that has a physics engine to simulate a brick falling, a holonomic turtle controller to control the turetle to the reference position, and a catcher node to organize turtle's behaviors so that it can catch the falling brick.

`turtle_brick_interfaces`: Customized message and service interfaces for `turtle_brick`

## Quickstart
1. Use `ros2 launch ros2 launch turtle_brick turtle_arena.launch.xml` to start the arena and turtle simulation
2. Use `ros2 service call /place turtle_brick_interfaces/srv/Place "{x: 6.54445, y: 7.544445, z: 7.0}"` and then `ros2 service call /drop std_srvs/srv/Empty "{}"` to drop a brick
3. Here is a video of the turtle when the brick is within catching range

https://github.com/user-attachments/assets/1362b876-16e6-4f83-8f06-ad321058b541

4. Here is a video of the turtle when the brick cannot be caught

https://github.com/user-attachments/assets/381ca313-03f0-48a9-8a66-f1349edca630
