# 2023Robot
The 7617 robot code for 2023. For our current code go to our [GitHub Page](https://github.com/roboblazers7617).

## Subsystems
Subsystems are used to group parts of the robot together and abstract low-level control. Subsystems include drivetrain, arm, and intake.

## Commands
Commands are used to operate the subsystems. Commands are triggered by the robot container and, when activated, take control of certain subsystems. This prevents multiple commands from trying to use the same subsystem. Commands include "drive to tag" and "lower intake".
