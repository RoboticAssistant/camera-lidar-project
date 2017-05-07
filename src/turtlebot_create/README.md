ROS package for iRobot Create 2 robot base
======

The ROS package contains drivers for iRobot Create 2 to allow USB to serial communication. The changes are made for opcodes and baud 
rate for the iRobot Create 2 specifications.

The following are the opcodes,

CREATE2_OPCODES = dict(
    soft_reset = 7,
    query = 142,
    pwm_motors = 144,
    drive_wheels = 145,
    drive_pwm = 146,
    stream = 148,
    query_list = 149,
    do_stream = 150,
    scheduling_leds = 162,
    digit_leds_raw = 163,
    digit_leds_ascii = 164,
    buttons = 165,
    schedule = 166,
    set_day_time = 167,
    stop = 173,
    )

The default baud rate for Create 2 base is 115200 bps.
    
    
    1) Clone this project to your catkin's workspace src folder
    2) Run catkin_make
