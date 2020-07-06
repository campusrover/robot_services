# VISION

## Questions

* is Redis the right way to continue to expand the API. I think so but a little research would be useful. Maybe we use it as a command queue for the new commands that I describe below.

## Long term vision objectives:

1. Allow non-roboticists to use Robots in their research
    1. including virtual and real robots
    1. Without requiring knowledge of ROS
1. Build a virtual / remote robotics lab
    1. Allow users from around the world to work with our robots
    1. Manage and allocate resources (e.g. robots)

## Virtual Lab Management Component (part of RSB suite)

1. "A lab with an API"
1. Authenticated users
1. Request and be assigned a robot
    1. Physical
        1. From a collection of robots that are owned and are ready to go
        1. Requesting human supervision (i.e. a lab tech physically there)
        1. Requesting overhead video
        1. Requesting a particular "field" to work in
    1. Virtual
        1. Request a particular inital environment from a set of available templates
        1. Request may include configuration parameters (e.g. size, initial location of robot, fiducials, etc.)

## RSB bindings (part of RSB suite)

1. Initially rsb_py
    1. Built by extraxting it from the pygame testbed that Pito's written
    1. Has no other dependencies
    1. Purpose is that user of the library doesn't have to learn REDIS

1. Future
    1. Unity?
    1. Javacript?

## rsbscript - Simple scripting abstraction (part of RSB suite)

1. The rest of Pito's RSB_PY
1. To Do
    1. Add a "dashboard" to show current status of robot
    1. Complete the lidar scan lines
    1. Improve performance
    1. Explore and design the new abstraction by writing:
        1. A program to do a roomba walk through a closed field
        1. A program to follow a wall
        1. A program to solve a maze


# NEW FEATURES
(assume all keys start with <ns>/)
**(@nate: please check off the ones that are done)**

1. [DONE] Change Key /Cmd -> /Kirby - because Kirby has a very specific set of behaviors which are at a higher level and stateful. 

---

2. [DONE] Change Key /Feedback to /Kirby/Feedback. Key  A subset of the messages coming out of Kirby are meant for "human" consumtion, report back the state and are needed to understand the result of the commands.

---

3. [DONE] Modify /Kirby/Feedback A message to include a "code": "xxx" key, e.g.

{"message": "unable to achieve goal", "code": "NO_GOAL"}

* The "code" is fixed and reliable; the message can be reworded as necessary without changing any code.

---

4. [IN PROGRESS] Add key /Cmd - Does a one of many commands. 

* Only one command at a time.
    * durations are in seconds
    * distances are in cm
    * speeds are in cm/second or degrees/second
    * angles are in degrees
* As the command is being executed the variable is changed to some indication of status (or progress)
* Once it is done the variable changes to success, invalid, stalled

Here the the initial three commands

a. {"cmd": "move", "duration": "1", "distance": "25", "speed": "25" } # Note that not all three can be supplied. Just two of the three and the third is computed

b. {"cmd": "rotate", "duration": "1", "angle": "90", "speed": "90" } # Note that not all three can be supplied. Just two of the three and the third is computed

c. {"cmd": "stop"}

---

5. [NEARLY DONE] Key /Status - Report the status of the robot, the environment and RSB itself. Not sure how this is implemented but it does an LPUSH to the key every 30 secconds that looks a  little like this. If all nodes are reporting that they are alive, then status is green and we just report the CPU % (to see if anything is looping infinitely.). If one or more nodes are not working, then the status is YELLOW with a list of nodes that seem broken.
 
    {time: "xxxx", "status": "GREEN", "CPU%": "80"}

    or 

    {time: "xxx", "status": "YELLOW", "errors": ["node1", "node2"], "cpu%": "80}
