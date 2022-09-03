.. _getting_started:

****************************
Getting Started
****************************

Here you will learn what is required to run the package successfully and in what order things should be started, including the intended software version and OS of the package.

============================
Software Versions
============================
This project was run on Ubuntu 20.04 and ROS melodic. 
The python version is 2.7.17.

============================
Installing Package
============================

Only ROS-specific and stanard python libraries are used. Therefore installing the package only requires the user to create a catkin_workspace and install using catkin. Follow the instructions on the ROS wiki to create a workspace <http://wiki.ros.org/catkin/Tutorials/create_a_workspace>.

Go to the workspace directory and use the below commands.

.. code-block:: bash

    $ catkin_make
    $ catkin_make install



==============================
Running nav_bot
==============================

Three commands need to be executed in three separate terminals. In the future these will be consolidated into a single execution script. Make sure the package is installed first, with the above commands.

The first script is ``fsm_action.py`` followed by ``start_experiment.py``.

* ``fsm_action.py`` is located in /nav_package/tb3_controller/scripts/
* To run ``fsm_action.py`` do the following.


.. code-block:: bash

    $ python fsm_action.py


* ``start_experiment.py`` is located in /scripts/
* To run ``start_experiment.py`` do the following.


.. code-block:: bash

    $ python start_experiment.py


Finally, run the ROS launcher with the folllowing.


.. code-block:: bash

    $ roslaunch nav_bot tb3_navigation.launch

