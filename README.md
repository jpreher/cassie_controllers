# Cassie Controllers

All of the controllers have been tuned for our hardware, you may need to modify some gains or offsets when first using the controllers. Most important control parameters are initialized in a [roslaunch file](https://github.com/jpreher/cassie_interface/tree/master/launch). A script is provided for tuning the CLF via the CARE solution in a [MATLAB file](https://github.com/jpreher/cassie_controllers/blob/master/MATLAB/createCLF_P_stand.m). The printed results can be copy/pasted into the corresponding launchfile parameters. 

Joystick commands are given as follows:

Joystick | Description
------------------------ | -------------------------
SA | Emergency stop
SB | -1=NULL / 0=Stand / 1=Walk
S1 | Swing leg pitch offset angle during walking
LH | Horizontal strafing
LV | Sagittal walking speed
RV | Turn rate for heading
LS | Standing height (keep at max for transition to/from walking)
SH | Trigger for predefined dynamic crouch if LS at max and in standing mode



When using our crouching and walking controllers on your hardware, please be aware that we have spent no time developing smooth transition behaviors for stand to walk and walk to stand. The robot will switch cleanly between these modes, but you will need to either code your own or provide some light manual support when going back and forth.
