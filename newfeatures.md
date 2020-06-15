NEW FEATURES
(assume all keys start with <ns>/)

1. Change Key /Cmd -> /Kirby - because Kirby has a very specific set of behaviors which are at a higher level and stateful. 

---

2. Change Key /Feedback to /Kirby_Feedback. Key  A subset of the messages coming out of Kirby are meant for "human" consumtion, report back the state and are needed to understand the result of the commands.

---

3. Modify /Kirby_Feedback A message to include a "code": "xxx" key, e.g.

{"message": "unable to achieve goal", "code": "NO_GOAL"}

The "code" is fixed and reliable; the message can be reworded as necessary without changing any code.

---

4. Add key /Cmd_vel - Does a Cmd/Vel operation for a certain duration:

{"duration": 1, "linear": 0.3, "angular": 0.5 }

Would mean 0.3 meters per second, 0.5 radians per second for 1 second.

---

5. Key /Status - Report the status of the robot, the environment and RSB itself. Not sure how this is implemented but it does an LPUSH to the key every 30 secconds that looks a  little like this:
 
 {time: "xxxx", "status": "OK", "CPU%": "80"}
