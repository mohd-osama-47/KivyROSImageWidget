# KivyROSImageWidget
Kivy widget that allows for embedding ```sensor_msgs/Image``` topics into your application.
The widget also stores the image as a cv2 frame and can be used directly for further image processing tasks

![Trace Demo](./media/ArmDemo.gif)
![Turtlebot Demo](./media/BotDemo.gif)

## What is this?
This arose due to the lack of resources for creating really slick and moder material design compatible GUIs with ROS. Kivy and KivyMD are great projects for creating modern looking mobile apps, but I have not found much when it comes to ROS projects.
This is an attempt from my side to slowly but steadly port over all my personal work and development I have done with creating modern apps that can interact your ROS network. Currently, this widget has been tested with ROS-1 Noetic, with more widgets for other tasks coming soon hopfully!

## How to use
The widget is stored in its own python file and you can just simply import it and use it as an ```Image``` widget in your ```.py``` or ```.kv``` files. I have embedded and example with the python file though you would need to run a MoveIt! configuration setup with the panda robot as stated in the MoveIt! tutorials site.
I will add more exampled with other robots just to show how flexible and pretty you can make your GUI's look for your robotic applications without sacrificing performance.
