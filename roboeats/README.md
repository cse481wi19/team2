# How to start PBD (Simulation)

Terminal 1:

``
roscore
``

Terminal 2:

``
roslaunch 
``

Terminal 3:

``
roslaunch roboeats pbd.launch cam_image_topic:=mock_point_cloud
``

Terminal 4:

``
rosrun applications hallucinator_demo.py data/tags.bag 
``

Terminal 5:

``
rosrun applications pbd_demo.py 
``