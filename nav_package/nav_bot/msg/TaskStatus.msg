std_msgs/Header header       # Message header
string robot_id              # Unique robot identifier
string task_id               # Unique task identifier
uint8 status
uint8 MOVING             = 0 # Robot moving toward the task
uint8 PAUSE              = 1 # Pause (e.g., to avoid a collision)
uint8 RESUME             = 2 # Resume from PAUSE
uint8 ARRIVED            = 3 # Robot has arrived at task
uint8 BEGIN              = 4 # Robot has begun executing the task
uint8 SUCCESS            = 5 # Task successfully executed
uint8 FAILURE            = 6 # Task failed
uint8 ALL_TASKS_COMPLETE = 7 # All tasks have been completed
uint8 AGENDA_CLEARED     = 8 # This robot's agenda has been cleared
uint8 ABANDONED          = 9 # Gave up on moving toward this task
