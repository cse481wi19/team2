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


# Creating nav markers

- Create the map first using ``roslaunch fetch_navigation build_map.launch``. 
  - Recall https://github.com/cse481wi19/cse481wi19/wiki/Lab-16%3A-Mapping-and-navigation
- Use our annotator demo to create a .pkl containing the poses we want.
  - ``rosrun applications annotator_demo.py ``
  - We need poses with these names:
    - ``microwave_location`` : location in front of the microwave
    - ``dropoff_location`` : location of the table where we drop off the food