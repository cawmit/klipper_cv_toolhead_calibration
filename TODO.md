# Big list of ideas to improve, some small goals, some big

- Be able to use T0, T1, ... macros etc instead of current toolhead switching code, also needed for use in toolchanger setups  
- Move T0 and T1 toolheads back to original position instead of current logic where it moves back to home position (Needed for first beta version)  
- Proper integration with requests lib for the streamer (Needed for first beta version)  
- Make opencv python import optional and only fail after printer start event with some proper checks (Needed for first beta version)  
- Figure out proper camera/toolhead rotation correction (Needed for first beta version)  
- Create 'clip on bed' USB microscope mount  
- Experiment with toolhead Z offset calibration: moving the camera up and down to detect if the nozzle is "in focus" in the image. USB microscopes have a very shallow depth of field, shallow enough? Requires solid camera mount  