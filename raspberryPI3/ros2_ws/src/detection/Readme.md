Detection package
=================

Summary
-------

This package adds obstacle detection feature.    
It adds :
* ``obstacles_detection`` node 
* ``/obstacles_detection`` topic

The node will publish a True message on the topic every time an obstacle is detected within 50 cm of the front-left or front-right sensors. Otherwise, it publish a false message.

The package contains :
* ``detection`` folder : it contains the node ``obstacles_detection.py``
* ``test`` folder : the differents tests done on the node
* ``ressource`` folder : where the executable is generated
* ``LICENSE`` file
* ``package.xml`` file : where you can find informations and dependencies for this package
* ``setup.py`` and ``setup.cfg`` : installation files

How to install
-------

Use the following command (be sure to be in `~/geicar/raspberryPI3/ros2_ws` before) :

* ```colcon build --packages-select "detection"``` to build this package
* ```source install/setup.bash"``` to source

How to use
---------

* To start the node : ```ros2 run detection obstacles_detection```

* to listen the topic : ```ros2 topic echo /obstacles_detection ```

How to test 
----------
There are two available test : 
- one unit test that tests the detection function
- one integration test to test the node, this one is still a work in progress

To run a test : 

* To run every test : ``colcon test --event-handlers console_direct+ --packages-select detection``

* To run just the unit test : ``colcon test --event-handlers console_direct+ --packages-select detection --pytest-args test/test_unit_obstacles_detection.py``