
## ROBOT_SERVICES: Movement Commands and Feedback 

### MOVEMENT AND COMMANDS: 

#### Basic commands: 

##### "go forward (X)"
* move to a space the specified number of meters ahead of where the robot currently is or a default of 1m ahead if none is specified
	* will fail if that location is blocked by or inside of a known obstacle. 
		* if it fails, the robot will rotate in place twice while attmpeting to find a path and then ultimately not move, if it cannot reach the desired location. 
	* if parts of the map are unexplored such that whether or not the desired location can be reached is unknown, the robot will move to explore unknown areas that could lead to a path to the desired location. 
		* if a path to the desired location is found, the robot will navigate to the desired location.
		* if no path is found, the robot will stop as soon as it is sure that there is no path to the desired location. 
	* although the command is 'go forward', the robot will not necessarily move in a straight line to the desired location
		* if there is a clear unobstructed path, it is likely it will go directly to the desired location
		* if a block, for example, is between the robot and the desired location, it will go around the block.
* inputting a negative value will generate a goal location behind the robot. 

##### "go to X Y"
* move to the desired x y coordinate on the map. 
	* functionality is identical to 'go forward' except that the goal location is specified by the input x y coordinates 
	* the x,y inputs are mandatory, not optional.

##### "turn left/right (X)"
* turn left or right the specified number of degrees or a default of 90 if none is specified
	* rotation is split into an estimation and verification step
	* most of the turn is completed and the goal position is estimated by directly setting his angular velocity and allowing him to turn for a specific amount of time. 
	* after this estimation a goal position is generated and sent to the same thing that handles navigation in 'go forward' and 'go to'. This is an attempt to make rotation as precise as possible. 
	* In the estimation phase, the goal location might not quite have been reached, or might have been overshot due to environmental factors, so this verification is an attempt at correcting any error or variance that may have occurred. 


#### Handling Multiple Commands: 

* The robot can take in consecutive commands which will be added to a queue and executed in the order in which they were input.  
* As another form of error correction, the location of each of the goals is set at the time it's received, with the resulting location of the previous command as the assumed reference point, rather than his real time position when he gets around to the point in the queue where he is completing that command
	* This is done intentionally, because there can be variance between the ideal goal and the actual performance. 
	* By using the ideal resulting location of any given goal to frame then next goal, rather than the actual location, which may have been over or under shot by a small factor, we hope to avoid the compounding of these small errors. 


#### Pausing, Stopping, and Cancelling:

##### "stop"
* if a stop command is received, the robot will immimmediately pause whatever it is doing. 
* A continue command must be received before the robot will restart taking in and executing commands
	* This means that it will not move, and any commands sent will not be added to the command queue until it has received a continue command

##### "continue" 
* if a continue command is received immediately after a stop command, the robot will simply continue with the goal it had been executing. If there are additional commands queued, it will execute them as normal once it finishes the current command. 

##### "cancel"
* a cancel command can only be given after a stop command. It cancels the current goal that was being executed before it was puased. 
* If only the current command is queued, then once the continue command is received, the robot will stay where it is, waiting for new commands. 
* If more commands are queued, then the one that was paused is deleted, and all subsequent commands are re-parsed from the perspective of this new and unexpected location. When the robot receives a continue command it will proceed, from this new location, with all queued commands.

##### "cancel all"
* a cancel all command can only be given after a stop command. It cancels all goals in the queue, including the current one. When a continue command is received the robot will stay where it is, waiting for new commands. 

##### "go back" 
* a go back command can be given after a stop command. The robot will cancel its current goal and any queued goals and return to the location from which it started the most recent goal. From there it will wait for additional commands, as any queued commands have been cancelled. 

#### Exploring Too Far: 

* As mentioned above, the robot might wander in an unexpected path if based on current map knowledge, the robot is unsure whether or not it can make it to a goal location it's received. 
	* sometimes, after exploration, it will realize that it definitely can't make it to the goal location, and will stop where it is, which is not the starting location, and also not the goal.
		* in these cases, the robot will check to see whether the distance it has traveled in its exploration is beyond some pre-set threshold. 
		* if it is beyond the threshold, it will wait for user input on how to proceed. 
			* the options from the user are "go back" and "keep going"
				* "go back" sends the robot back to the location it was in when he started exploring in an attempt to find a path to his goal. From there he will continue with any queued actions
				* "keep going" tells it to just continue any queued actions from the new locaiton. 
		* if the new location is within the threshold, it will simply continue with queued actions from this new location. 


#### Patroling: 
	
* The robot can also patrol/explore its environment. 
* It will begin this behavior with a "patrol (points) (initial radius) (increment)" command
	* A patrol command overwrites all existing commands. The robot will immediately stop any goal that it is executing, and cancel any queued goals in order to begin patrolling. 
* The robot explores in concentric "circles", visiting some fixed number of points along the edge of each circle. 
	* It begins relatively near the origin, and the circles expand outward. 
* If no parameters are passed in with a 'patrol' command, it will explore a default of 16 points around each circle. The first circle will have a radius of 1.5m centered at the origin, and the radius of each subsequent circle will be incremented by 1m. 
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

##### go forward:
* After it begins executing a go forward command: 
	* "currently looking for a path forward <X>m to (x, y)"
	* the <X> is the number of meters specified in the command, or 1.0 if no measure was specified
	* the x,y coordinate is the location that he has generated as the end goal, based on the direction he was facing when he started, and the number of meters he is meant to traverse. 
* After successful completion of a go forward command: 
	* "successfully went forward <X>m to (x, y)"

##### turn left/right:
* After it begins the rotation estimation portion of a turn command:
	* "estimating a <X> degree turn to the left/right"
	* <X> is the input number of degrees or 90 if none was specified
* After the estimation portion concludes:
	* "successfully estimated rotation"
* At the beginning of the verification portion of a turn command:
	* "verifying rotation"
* At the end of the verification portion:
	* "rotation verified"

##### go to:
* After it begins execution of a go to command:
	* "currently looking for a path to (x, y)"
	* x and y are the input x,y coordinates in the go to command
* After completion: 
	* "successfully navigated to (x, y)"

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

**note** 
* any time patroling stops, the main movement node needs to know it can retake control and start listening for commands again, so it listens for a 'terminating' status and when it receives this, retakes control and publishes "waiting for commands". 
* So a "waiting for commands" status after a "terminating" status signifies that the robot is ready to take in normal, non-patrol commands again. 

##### stop:
* If the current goal is a rotation estimation
	* "paused rotation estimation"
	* "rotation completed so far: <X> degrees"
		* where <X> is the numebr of degrees completed so far in this turn
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
	* "previous location (x, y)"
		* where x, y are the coordinates of the location of the robot at start of the paused goal
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


---
Summary of Commands

| Command | Effect | Notes |
|---|---|---|
|patrol | initiate the patrolling algorithm | a sophisticated algorithm to try to explore the whole reachable space |
| go to x y | go to coordinates x y | Works within navigability. From where you are try to plot a route to x and y. Uses navigation.
| turn left/right | turn the robot | Same
| stop | stop current processing | Commands are executed in sequence. More like a "pause" |
| continue | resume from a stop | 
| cancel | cancel stopped processing | ?
| go forward | move forward | Optional parameters can control the distance. Works within the constraints of navigability.
---
 
#### Notes
* These commands maintain a notion of "previous location" to which they can be returned.

# Status log messages
* Returning to previous location
* Returned to previous location
* Patrolling cancelled
* Unable to complete goal


