NEW FEATURES
(assume all keys start with <ns>/)

1. Change Key /Cmd -> /Kirby - because Kirby has a very specific set of behaviors which are at a higher level and stateful. 

---

2. Change Key /Feedback to /Kirby/Feedback. Key  A subset of the messages coming out of Kirby are meant for "human" consumtion, report back the state and are needed to understand the result of the commands.

---

3. Modify /Kirby/Feedback A message to include a "code": "xxx" key, e.g.

{"message": "unable to achieve goal", "code": "NO_GOAL"}

* The "code" is fixed and reliable; the message can be reworded as necessary without changing any code.

---

4. Add key /Cmd - Does a one of many commands. 

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

5. Key /Status - Report the status of the robot, the environment and RSB itself. Not sure how this is implemented but it does an LPUSH to the key every 30 secconds that looks a  little like this. If all nodes are reporting that they are alive, then status is green and we just report the CPU % (to see if anything is looping infinitely.). If one or more nodes are not working, then the status is YELLOW with a list of nodes that seem broken.
 
    {time: "xxxx", "status": "GREEN", "CPU%": "80"}

    or 

    {time: "xxx", "status": "YELLOW", "errors": ["node1", "node2"], "cpu%": "80}
