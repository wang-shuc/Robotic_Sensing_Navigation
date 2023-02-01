This is a README file.
Shuchong Wang
EECE 5554 LAB 2

For commands to run the GPS driver.
0. Make sure you have a "roscore" running, and gps port plug in with address "/dev/ttyACM0"
    If you have a different port address/location, please change it inside the python script,
    otherwise it will keep generating error
	"No such file or directory: '/dev/ttyACM0'"

1. in another terminal, go to the LAB2 directory
	"cd LAB2"

2. since there is only a src directory, do a 
	"catkin_make" 
    under LAB2 directory, the make should success without any problems.

3. then do 
	"source ./devel/setup.bash"

4. dive into the ros package
	"roscd gps"

5. to run the python script (the gps driver)
	"rosrun gps driver.py"

*Additional Support"
If you didn't grant permission to gps port, it might fail to compile. Do the following:
	"sudo chmod 666 /dev/ttyACM0"
typing the password for your Ubuntu, then everything should be fine afterwards.

-------------------------------------------------------------------
----The remaining parts of README file is lab explanation----
-------------------------------------------------------------------
Thanks for reading/grading my lab2. 
I didn't write any instructions/commands to run the gps driver for lab1 inside its README file, and lab1 is past due. 
Therefore, I think I cannot make any changes now.
For this lab2, I wrote commands above, please take a look.
The package name is similar to lab1, I use "gps" for ros package it's the same as lab1.
Also the "driver.py" is mirrored by "gps_driver.py" from lab1.
The only modification for "driver.py" is changing the file to parsing from "GPGGA" to "GNGGA"
If you run the commands above correctly, I believe there will be some print-outs in the terminal (EVEN IF there's NO signal for gps sensor, the terminal will display some information)

Two datasets are collected from different locations. 
One is from MacDonald Stadium next to Malden Center MBTA station.
This is the data for completely clear spot.
I think a playground is very clear without any surrounding or shading.
The other one is from Northeastern University next to the West Village Hall.
This spot is for data that is partially occlusion and reflection nearby.
Because at that spots, there are buildings and trees. 
I also mentioned this part in my lab report.

The data are all recorded using "rosbag record -a" commands.
Afterwards, we firstly check the bag information "rogbag info <bagfile name>"
Then we convert our bagfile into yaml format.
The python can read yaml file directly without any issue.
Inside analysis scripts, I commented out many lines.
If you want to see different plots, you may want to uncomment some line, choose the correct file you desired and corrent plot you wanted.