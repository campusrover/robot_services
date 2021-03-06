
## ROBOT_SERVICES: Movement Commands and Feedback 

### MOVEMENT AND COMMANDS: 

#### Basic commands: 

##### "go forward (X)"
* Move to a space the specified number of meters ahead of where the robot currently is or a default of 1.0 m ahead if no distance is specified.
* Go forward commands are completed with move_base waypoints. 
	* Will fail if that location is blocked by or inside of a known obstacle. 
		* If it fails, the robot will rotate in place twice while attempting to find a path and then ultimately not move, if it cannot reach the desired location. 
	* If parts of the map are unexplored such that whether or not the desired location can be reached is unknown, the robot will move to explore unknown areas that could lead to a path to the desired location. 
		* If a path to the desired location is found, the robot will navigate to the desired location.
		* If no path is found, the robot will stop as soon as it is sure that there is no path to the desired location. 
	* Although the command is 'go forward', the robot will not necessarily move in a straight line to the desired location
		* If there is a clear unobstructed path, it is likely it will go directly to the desired location
		* If a block, for example, is between the robot and the desired location, it will navigate around the block.
* Inputting a negative value will generate a goal location behind the robot. 

##### "go to X Y"
* Move to the desired x y coordinate on the map, again using a move_base waypoint.
	* Functionality is identical to 'go forward' except that the goal location is specified by the input x y coordinates 
	* The x,y inputs are mandatory, not optional.
	* They can be positive or negative, and represent ROS coordinates
	* They should be separeated by a space 

##### "turn left/right (X)"
* Turn left or right the specified number of degrees or a default of 90, if none is specified
	* X must be positive
	* Rotation is split into an estimation and verification step
		* Most of the turn is completed and the goal position is estimated by directly setting the robot's angular velocity and allowing it to turn for a specific amount of time. 
		* This is the only part of the robot's navigation that is done without move_base.
		* After this estimation, a goal position is generated, based on the starting position and the desired rotation, and sent to move_base. This as a form of error correction, since rotation estimation using only assumed velocity and timing can be imprecise, particularly in the real world where environmental factors may impact movement.  


#### Handling Multiple Commands: 

* The robot can take in consecutive commands which will be added to a queue and executed in the order in which they were input.  
* As another form of error correction, the location of each of the navigation goals is set at the time it's received, with the resulting location of the previous command as the assumed reference point, rather than the robot's real time position when it gets around to any given command in the queue.
	* This is done intentionally, because there can be variance between the ideal goal location and the actual result. 
	* By using the ideal resulting location of any given goal to frame then next goal, we hope to avoid the compounding of small errors, like the slight under or overshooting of a goal location. 


#### Pausing, Stopping, and Cancelling:

##### "stop"
* If a stop command is received, the robot will immediately pause whatever it is doing. 
* A continue command must be received before the robot can restart taking in and executing commands
	* This means that it will not move, and any commands sent will not be added to the command queue until it has received a continue command

##### "continue" 
* If a continue command is received immediately after a stop command, the robot will simply continue with the goal it had been executing. If there are additional commands queued, it will execute them as normal once it finishes the current command. 

##### "cancel"
* A cancel command must follow a 'stop' command.
* It cancels the current goal that was being executed before it was puased. 
* If only the current command is queued, the robot will stay where it is, waiting for new commands. 
* If more commands are queued, then the one that was paused is deleted, and all subsequent commands are re-parsed from the perspective of this new and unexpected location. The robot will proceed, from this new location, with all queued commands.

##### "cancel all"
* A cancel all command must follow a 'stop' command.
* It cancels all goals in the queue, including the current one. The robot will stay where it is, waiting for new commands. 

##### "go back" 
* A go back command must follow a 'stop' command.
* The robot will cancel any queued goals and return to the location from which it started the most recent goal. From there it will wait for additional commands. 

#### Exploring Too Far: 

* As mentioned above, the robot might wander in an unexpected path if based on current map knowledge, it is unsure whether or not it can make it to a goal location it's received. 
	* Sometimes, after exploration, it will realize that it definitely can't make it to the goal location, and will stop where it is, which is not the starting location, and also not the goal.
		* In these cases, the robot will check to see whether the distance it has traveled in its exploration is beyond some pre-set threshold. 
		* If it is beyond the threshold, it will wait for user input on how to proceed. 
			* The options from the user are "go back" and "keep going"
				* "go back" sends the robot back to the location it was in when it started exploring in an attempt to find a path to the goal. From there it will continue with any queued actions
				* "keep going" tells it to just continue any queued actions from the new locaiton. 
		* If the new location is within the threshold, it will simply continue with queued actions from this new location. 


#### Patroling: 
	
* The robot can also patrol/explore its environment. 
* It will begin this behavior with a "patrol (points) (initial radius) (increment)" command
	* A patrol command can be queued like any other command. Once the robot reaches the patrol command in its queue of planned actions, it will begin patrolling. 
* The robot explores in concentric "circles", visiting some fixed number of points along the edge of each circle. 
	* It begins relatively near the origin, and the circles expand outward. 
* If no parameters are passed in with a 'patrol' command, it will explore a default of 8 points around each circle. The first circle will have a radius of 1.0m centered at the origin, and the radius of each subsequent circle will be incremented by 1.0m. 
	* If parameters are passed in, all 3 must be specified. 
		* The first is an integer representing the number of points to explore per circle
		* The second represents the radius of the initial circle
		* The third represents the amount by which the radius of subsequent circles should be incremented
* The robot begins patrolling by generating the initial circle based on the input or default initial radius. 
	* It generates the input or default number of points along the edge of the circle. 
	* It attempts to vist each one, in a counterclockwise direction. 
		* If any point is inaccessible, it will determine that it cannot reach that point, and simply move on to the next one. 
* Once it has explored all points in the initial circle, it will generate a new circle, larger (by the addition of the increment) than the first, and new points to explore around it. 
	* This behavior will continue until all of the points generated are inaccessible, at which point it will report that there is no where left to explore, and stop patroling 
* Patroling can also be stopped with a "stop patrol" command. 
	* After patrolling is stopped, the robot is again ready to take in commands


#### FEEDBACK: 

* The robot publishes a log of its actions, every time it does or tries soemthing. 
* The robot has various feedback codes representing different states that it can be in. Each feedback code is also accompanied by a messge. The codes are listed in a table below, and the messages are described here. 

##### go forward:
* After it begins executing a go forward command: 
	* "going forward <X>m to (x, y)"
	* the <X> is the number of meters specified in the command, or 1.0 if no measure was specified
	* the x,y coordinate is the location that has been generated as the end goal, based on the direction the robot was facing when it started, and the number of meters it is meant to traverse. 
* After successful completion of a go forward command: 
	* "went forward <X>m to (x, y)"
* If it receives a go forward command with more than one argument:
	* "go forward commands only take one argument"

##### turn left/right:
* After it begins the rotation estimation portion of a turn command:
	* "estimating a <X> degree turn to the left/right"
	* <X> is the input number of degrees or 90 if none was specified
* After the estimation portion concludes:
	* "estimated rotation"
* At the beginning of the verification portion of a turn command:
	* "verifying rotation"
* At the end of the verification portion:
	* "rotation verified"
* If it receives a rotation command with a negative angle
	* "angle must be positive"
* If it receives a rotation command with more than one argument
	* "rotation commands only take one argument"

##### go to:
* After it begins execution of a go to command:
	* "going to (x, y)"
	* x and y are the input x,y coordinates in the go to command
* After completion: 
	* "navigated to (x, y)"

##### patrol: 
* After it receives a patrol command:
	* "creating <X> waypoints around a circle with radius <Y>"
	* <X> is the input number of waypoints, or the default
	* <Y> is the input initial radisu, or the default
* Once it has planned the exploration path and begun patrolling: 
	* "patrolling"
	* If patrolling is stopped with a 'stop patrol' command:
		* "patrolling cancelled"
		* "terminating"
	* If patrolling is stopped because there is no more area to be explored: 
		* "no more explorable waypoints"
		* "terminating"
* Once it completes one exploration circle
	* "completed an exploration loop"
* If it receives a patrol command with a number of arguments not equal to 3 or 0
	* patrol must take 3 arguments or none at all"


##### stop:
* If the current goal is a rotation estimation
	* "paused rotation estimation"
* If the current goal is anything else
	* "paused current goal"

##### continue:

* If the previous command was 'stop':
	* "restarting current goal"
* If the previous command was 'cancel' and there are pending goals in addition to the one that was paused:
	* "cancelled current goal"
	* "continuing execution of planned goals"
* If the previous command was 'cancel' and the paused goal is the only goal queued:
	* "cancelled goal"
	* "waiting for commands"
* If the previous command was 'cancel all' and there are pending goals:
	* "cancelled all planned goals"
	* "waiting for commands"
* If the previous command was 'cancel all' and there are no pending goals:
	* "cancelled goal"
	* "waiting for commands"
** note **
	* The robot will not publish feedback after receiving a 'cancel' or 'cancel all' command on its own, since it must also receive a continue command before it can do anything else. As shown above, the status will be published after a continue command is received. 

##### go back:

* If a go back command is received after a stop command:
	* "returning to previous location"
* Once the robot has made it back to the location:
	* "returned to previous location"

##### failure: 
* If the robot fails to make it to any given goal, he will check whether or not it has moved in an attempt to get there.
	* If it has moved beyond the threshold: 
		* "moved from expected path and failed to reach goal"
		* "user input is required: keep going OR go back"
			* If user input is 'keep going':
						* "continuing from new location"
			* If user input is 'go back':
						* "returning to previous location"
	* If it has not moved beyond the threshold: 
		* "unable to complete goal"

##### invalid commands: 
* If the robot receives an invalid/malformed command it publishes an invalid message to the log. 
* This can take several different forms, depending on the problem with the input message: 
	* "invalid command, robot is paused"
	* "invalid command, robot is patrolling"
	* "robot must be stopped before a goal can be cancelled"
	* "robot must be stopped before it can continue"
	* "invalid command" 

#### FEEDBACK CODES: 

| Code | Meaning | Notes |
|---|---|---|
| READY | the robot is waiting to take in commands 
| INVALID | the input command is invalid
| PAUSED | the robot has paused whatever it was doing 
| RESTARTING | the robot is restarting any queued or in-progress goals, after being paused
| CANCELLED_GOAL | the robot cancelled the goal it had been executing
| CANCELLED_ALL | the robot cancelled all planned goals
| FORWARD | the robot is executing a "forward" command
| GO_TO | the robot is executing a "go to" command
| ESTIMATE_ROTATION | the robot is completing step 1 of a "turn left/right" command
| VERIFY_ROTATION | the robot is completing step 2 of a "turn left/right" command
| GO_BACK | the robot is going back to its previous location
| SUCCESS_FORWARD | the robot successfully completed a 'go forward' goal
| SUCCESS_GO_TO | the robot successfully completed a 'go to' goal
| SUCCESS_ESTIMATE_ROTATION | the robot successfully estimated the rotation necessary to complete a 'turn' goal
| SUCCESS_VERIFY_ROTATION | the robot successfully verified its rotation estimation for a 'turn' goal
| SUCCESS_GO_BACK | the robot successfully went back to its previous location
| UNREACHABLE | the robot cannot complete the goal 
| STRAYED | the robot moved from it's original location but was unable to complete its goal
| HELP | the robot requires user input 
| PATROL | the robot is patroling
| PLAN_LOOP | the robot is planning its next exploration loop while patrolling
| COMPLETED_LOOP | the robot completed an exploration loop while patrolling
| STOP_PATROL | the robot has received a command to stop patrolling 
| FINISH_PATROL | the robot has no more unexplored waypoints, the patrol algorithm has finished
| QUEUE | all commands that are currently in the robot's planned actions

---
Summary of Commands

| Command | Effect | Notes |
|---|---|---|
|patrol | initiate the patrolling algorithm | an algorithm to try to explore the whole reachable space |
| stop patrol | stop the robot's patrolling algorithm | 
| go to x y | go to coordinates x y | Works within navigability. From where you are try to plot a route to x and y. Uses navigation.
| turn left/right | turn the robot | Same
| stop | stop current processing | Commands are executed in sequence. More like a "pause" |
| continue | resume from a stop | 
| cancel | cancel the action that was paused with a stop command | Can only be used following stop command |
| cancel all | cancel any and all queued actions | Can only be used after a stop command
| go forward | move forward | Optional parameters can control the distance. Works within the constraints of navigability.
| go back | cancels the current action and returns to the location the robot was in when it started the cancelled action | can only be used after a stop command
| queue | the robot will publish feedback containing its current queue of planned actions
| go back AND/OR keep going | possible responses to the robot's request for user input | these commands are only valid when robot requests user input. This will only happen if the robot moves from its original location in an attempt to find a path to a goal location, but ultimately determines that the goal location cannot be reached, and therefore the robot stops in an unexpected location. 
---
 

#### Notes
* These commands maintain a notion of "previous location" to which they can be returned.

# Status log messages
* Returning to previous location
* Returned to previous location
* Patrolling cancelled
* Unable to complete goal


