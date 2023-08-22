# Toy project

## Description
Fun control code that can be used to drive a common robot, e.g. Turtlebot3 (specifically the burger), to an exact location from anywhere on a given map. A home function drives the robot back to the starting location.

## docs
Docs were generated using sphinx. To view the documation on a web browser go to docs/_build/html/ and open index.html in your favourite browser.

## Code in nav_package/nav_bot and nav_package/tb3_controller and scripts
For full description please open index.html as mentioned above.

## To run the project
Open 3 separate terminals. 

* First terminal 

Go to the file location of **fsm_action.py** in nav_package/tb3_controller/
```
$ python fsm_action.py
```

* Second terminal, follow the onscreen instructions on this terminal to control the turtlebot3

Go to the file location of **start_experiment.py** in scripts/
```
$ python start_experiment.py
```

* Third terminal, make sure the package was installed with catkin before attempting this

```
$ roslaunch nav_bot tb3_navigation.launch
```

New change - work in progress
