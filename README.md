# moplan
Two link arm simulator. Implements:
* Controllers: PID
* Motions: Holding target position, following linear C-space trajectory
* Interaction: Reacting to forces applied to end-effector

To start simulator:
```
python 2linksim/run.py -flag
```

Pass the following flags for different functionality:
* -r : moves to randomly generated configurations and holds position
* -l : follows linear trajectory from start to goal configuration
* -f : applies an interaction force at the end-effector
* -i : performs inference on external force (TBA)
