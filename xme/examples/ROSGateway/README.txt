========================================
  CHROMOSOME Example "ROSGateway"
========================================

Node Name        | Platform
-----------------+---------
master           | Linux
ros_system       | Linux

Description
-----------
ROSGateway shows an example of how to connect CHROMOSOME with ROS. Therefore,
it shows how messages can be exchanged in both directions.
It contains an example for bidirectional message transfer (message and status).
In addition, client-server-communication (request/response in CHROMOSOME and
services in ROS) is shown from ROS to CHROMOSOME for the calculation of
differences. The inverse client-server-communication is demonstrated for the
calculation of sums.

Compilation
-----------
To start compile ROSGateway you first need to install ROS on your machine. For
how to install ROS see http://wiki.ros.org/ROS/Installation. The nodes can be
compiled by
1) typing cmake -G "Unix Makefiles" ../../src/application/ros_system
   for the ros_system and then make
2) typing cmake -G "Unix Makefiles" ../../src/application/master
   for the master and then make

Usage
-----
You can start the application by
1) starting roscore
2) starting ros_system (ROSGateway/build/ros_system/devel/lib/ros_system/
   ros_system)
3) within 10 seconds after starting ros_system the master needs to be started
   (ROSGateway/build/master/target/master)
