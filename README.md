****************************************************
**************       INTRODUCTION      *************
****************************************************

This repository contains the MAchine and Environment Software Translation Over Ros (MAESTOR) software project.
Included are install scripts to generate the execution environment required for MAESTOR, 
as well as an install script to generate the execution environment for virtualization and visualization of the HUBO robot.

****************************************************
**************       REQUIREMENTS      *************
****************************************************

- Ubuntu 12.04 LTS
- ROS Fuerte
- OpenRAVE (ROS stack)

****************************************************
**************       INSTALLATION      *************
****************************************************

Installation scripts exist to install dependencies such as ROS, OpenRAVE, Hubo-Ach and OpenHUBO.

****** 
 Note
******
 Pay strict attention to permissions for all scripts. Some require root access, and some mandate non-root access. ****

To install all dependencies and complementary packages.
	./install-all.sh

Alternatively, you can install some components individually:

Installation of ROS:
	sudo ./install-ros-fuerte.sh

Installation of OpenRAVE:
	sudo ./install-openrave.sh

Installation of Hubo-Ach:
	sudo ./install-hubo-ach.sh
	
Installation of OpenHUBO:
	./install-openHubo.sh
	
******
 Note
******
After installation, it is recommended for the user to interact with these packages in a new terminal, in order to 
ensure that environment variables are properly loaded. If any errors occur, please reboot the operating system and try again,
to confirm that the errors are not related to unloaded environment variables.

	
****************************************************
**************      UNINSTALLATION     *************
****************************************************

Navigate to <MaestOR Install Directory>/install/uninstall
Run:
	source uninstall.sh

You will be prompted to uninstall each of the separate components installed by the install scripts. 
NOTE: Using source to call the script is necessary because otherwise it will unable to read the OPENHUBO_DIR environment variable.

****************************************************
**************        QUICK START      *************
****************************************************

If you want to get started immediately, and read about the full interface later, follow these steps:


	- Run the maestor run script: maestor <run type>
		For example, if running in simulation:
            maestor sim
        or, on hardware:
            maestor real
		
	- If you chose "real" as your run type:
	
		- A terminal should have opened on tty7 (This may be different if there are already processes running on these terminals.)
		
		- Open tty7 (Ctrl - Alt - F7) and make sure the hubo-ach console is running (It may require elevated privileges.)

	- If you chose "sim" as your run type:
		
		- X Terminals should have opened with hubo ach, roscore and additional simulation windows. Hubo-Ach's terminal may require elevated privileges. Give them. On occation one of the xterminals will be directly over top of the other. If this happens and the one running hubo-ach that needs elevated permissions is on the bottom simply move the top one and give hubo-ach permission. 
		
	- Once everything has finished starting up, the terminal you started maestor in will be back in your control. 
	
	- From there, type in the following command:
		maestor console

	-This will give you a python console that has the maestor api and commands loaded. The function ls() will give you a list of all the functions you have available. 

	-To begin controlling the robot or the simulation you must first initialize the robot:
		
		>>> initRobot("")          #The parameter is the path to a config file. Empty string is default

	- If running on hardware, the robot's joints should all move through their homing routines.
		
		>>> command("HomeAll", "")

    - In order to move the joints them must be enabled. This can be done with the command EnableAll
	
		>>> command("EnableAll", "")
			
	- From here the robot in hardware or simulation can move all of it's joints. A simple test is to move the right arm up 

		>>> setProperty("RSP", "position", -1)
		
	- The robot should move its arm up. This should give you an indication of what Maestor can do.
		For more information, please visit the Interface section.
	
		
****************************************************
**************        INTERFACE        *************
****************************************************

****************************
**********  RUN  ***********
****************************

To run maestor type the command maestor and an argument:

Usage: maestor <Command> 
<Command>:
	sim: Starts maestor with hubo-ach in simulation mode with physics
	sim-real: Start maestor with hubo-ach in simulation mode without physics (Real time)
	real: Starts maestor with hubo-ach in hardware mode
	console: Starts the maestor python console
	kill: Kills maestor, hubo-ach, and roscore

Terminals will open which hold the following packages:
hubo-ach (Will require privilege escalation)
openHUBO (OpenRAVE Visualization of the HUBO Robot, optional)

This option allows a user to visualize the HUBO Robot's motion as commanded by MAESTOR, or directly control the robot on robot hardware.
The visualization package utilizes an OpenRAVE plugin simulation environment called OpenHUBO to display the simulation.Hubo-Ach controls raw communication with the Robot and simulation, 
and accepts commands through MAESTOR. 
MAESTOR provides services and thus, MAESTOR commands are funneled to the simulation.

The differences between each mode of operation are as follows:
	Virtual: Hubo-Ach runs in a virtual CAN environment. A separate OpenHUBO simulation appears, with no physics.
	Sim: Hubo-Ach runs in simulation mode, and opens its own simulation. Physics are enabled - positions of motors can be polled.
	Sim-real: Hubo-Ach runs in simulation mode without physics. This option is meant for previewing intended robot motion, and should be much faster.
	real: Hubo-Ach runs in hardware mode. This option is meant for an environment with no X-Server installed. No simulation is opened. A hardware CAN interface is expected.
****************************
*********  USAGE  **********
****************************

******************
**INITIALIZATION**
******************

Initialize Robot:
	initRobot("<Init File Path>");

The default directory which this method will fall back to if given an empty string is 
/opt/ros/fuerte/stacks/maestor/models/hubo_default.xml.

This command loads Robot configuration information from the xml file specified. 
Before this command, MAESTOR cannot perform any operations which work with actual robot components (Which all of the following commands do.)


Home All Joints:
	command("HomeAll", "");

Home A Single Joint:
	command("Home", "<Joint Name>");
	
This command will send the homing command down on the Ach channel, causing the joints to move to their home positions. Before this
command is run, no joints can move, nor should they. Each joint should move toward the "home" position on their respective axis, until
their physical parts hit a limit switch which lets them know that they have hit their home position. If, after running a homing command,
it appears that a joint has not correctliy homed, it is advisable to reset that joint and try homing again:


Zero All Joints:
    command("ZeroAll","");

Zero a Single Joint:
    command("Zero", "<Joint Name>");

This command will send the given joint, or all joints, to the zero position. If these joints have previously been homed, and are currently
enabled, they should begin to move to the position they were at after homing. This command is especially useful if the motors have moved beyond
the stopping point for their homing routines such that homing the motor could potentially harm the physical joint.


Reset All Joints:
	command("ResetAll", "");
	
Reset A Single Joint:
	command("ResetJoint", "<Joint Name>");
	
This command will reset the internal state of the motor board in the given joint or joints. This command should reset Big Error, Encoder Error, 
and Homing Error on the given joints, and potentially other hardware errors as well.


Initialize Sensors:
	command("InitializeSensors", "");
	
This command will initialize all of the sensors on the robot, including the FT and IMU sensors. The robot should be off the ground and not
moving when this command is run. This command should be run as part of initialization.


Set an alias:
	setAlias("<Entity Name>", "<New Name>");
	
This command will set an alias for some motor, sensor, property, or command. This will allow you to specify your own names for direct control,
or you can shorten the names programmatically to shorten the lengths of written scripts. An excellent use of this feature would be to have
a gesture created for initialization tasks that aliases your commands to the chosen values at the start.

A list of all default joint and sensor names is listed under the Default Names section. 
Properties are listed in the "GetProperties" method entry in the Feedback section.
The names of every command listed above can also be aliased.

Note: This command should be performed after running initRobot, so all entity names will be available to alias.


******************
**   MOVEMENT   **
******************

Move A Single Joint:
	setProperty("<Joint Name>", "Property", value);
Examples:
	setProperty("RHY", "position", 3.14);
	setProperty("RHR", "velocity", .2);
	setProperty("RHP", "motion_type", 0);

The setProperty command allows the user to set the values of certain properties of joints. Currently, the only properties that are mutable
are position, velocity, and motion type.. Position and velocity are measured in radians and radians per second, respectively. Motion Type is an enumeration
which is defined as the following values:

0 - Filtered motion. Hubo-ach will internally filter and interpolate the goal position.
1 - Non-filtered motion. Hubo-ach will set the motor reference directly.
2 - Compliant control. Compliance will be enabled for the motor. (Not confirmed working.)
3 - Compliant-encoder control. Hubo-ach will make the reference follow the encoder value. (Not confirmed working.)
4 - Reference Differential Control. Goal positions will be interpreted as deltas, instead of end positions. (Only for finger and neck boards.)

In addition, a user can set multiple properties of joints at once, by delimiting joints, positions, and values by either tabs, spaces, or newlines.

Examples:
	setProperties("RHY RHR LHY LHR", "position position position position", "1.49 1.49 1.49 1.49");
	setProperties("RHY RHY LHY LHY", "position velocity position velocity", ".2 1.49 .2 1.49");
	
******************
**   FEEDBACK   **
******************

Request a value from a Joint:
	getProperty("<Joint Name>", "Property");

Request multiple values from multiple joints:
    getProperties("<Joint Names>", "Properties");
	
The getProperty command allows the user to get the values of certain properties of robot objects. The following values can be retrieved:

From Joints:
position			- Position, in radians
velocity			- Velocity, in radians/sec
current				- Current
temperature			- Temperature
enabled				- Will return 1 if this joint has motion enabled
homed				- Will return 1 if this joint has been homed
errored				- Will return 1 if this joint has an error condition
jamError			- Will return 1 if this joint is experiencing a jam error
PWMSaturatedError	- Will return 1 if this joint is experiencing a PWM Saturated error
bigError			- Will return 1 if this joint is experiencing a big error
encoderError		- Will return 1 if this joint is experiencing an encoder error
driveFaultError		- Will return 1 if this joint is experiencing a Drive Fault error
posMinError			- Will return 1 if this joint is experiencing a pos (min) error
posMaxError			- Will return 1 if this joint is experiencing a pos (max) error
velocityError		- Will return 1 if this joint is experiencing a velocity error
accelerationError	- Will return 1 if this joint is experiencing an acceleration error
tempError			- Will return 1 if this joint is experiencing a temperature error

From FT Sensors:
m_x					- Will return the moment in the x-axis
m_y					- Will return the moment in the y-axis
f_z					- Will return the force in the z-axis

From IMU Sensors:
x_acc				- Will return the acceleration in the x-axis
y_acc				- Will return the acceleration in the y-axis
z_acc				- Will return the acceleration in the z-axis
x_rot				- Will return the rotation angle in the x direction
y_rot				- Will return the rotation angle in the y direction

******************
** TRAJECTORIES **
******************

A trajectory is defined as a chain of non-zero length of Whitespace Separated Value files. Each file may contain columns of
position values to be sent sequentially to joints which may be indicated by an optional header. A default header assumes
that all 40 joints will be included in the following order:
RHY RHR RHP RKP RAP RAR LHY LHR LHP LKP LAP LAR RSP RSR RSY REP RWY RWR RWP LSP LSR LSY LEP LWY LWR LWP NKY NK1 NK2 WST RF1 RF2 RF3 RF4 RF5 LF1 LF2 LF3 LF4 LF5

To load a trajectory, the following command is used:
    loadTrajectory("<Trajectory Name>", "<path to trajectory>", <playback>)
        Where Trajectory Name is a short name to be used to refer to that trajectory later, and playback is a boolean
        value that should be passed as true if the trajectory is to be read from and false if the trajectory is to be written to.
       
To run a trajectory that is loaded:
    startTrajectory("<Trajectory Name>")
    
To stop a running trajectory:
    stopTrajectory("<Trajectory Name>")
    


In addition, Trajectories have the following useful features:

To force a trajectory to ignore an input from one of its named columns, use this command:
    ignoreFrom("<Trajectory Name>", "<Column Name>")
    
To unignore a previously ignored column:
    unignoreFrom("<Trajectory Name>", "<Column Name>")
    
To ignore or unignore all columns:
    ignoreAllFrom("<Trajectory Name>")
    unignoreAllFrom("<Trajectory Name.")

To append a trajectory file to the end of a loaded trajectory*:
    extendTrajectory("<Trajectory Name>", "<path to trajectory>")
    
*a trajectory may only be extended if the new trajectory has the same number of columns, the same columns present (not 
necessarily in the same order), and must start relatively close to the last position vector of the current trajectory.
Ignoring extra columns from loaded trajectories may be used to extend trajectories with shorter columns.

To set a trigger for another loaded trajectory* to be played at a certain frame**:
    setFrame("<Trajectory Name>", <frame number>, "<Trajectory Name to be Loaded>")
    
*this method can be used to have a trajectory start itself when the trajectory has finished
*the "frame" of the trajectory is the number of rows of data which have been read. Comments/headers/empty lines do not count.
A user can specify a frame of -1 to load another trajectory at the end of the given trajectory.

****************************************************
**************          DEMOS          *************
****************************************************

To run hubo demos with maestor, download them from the github repositories: 

https://github.com/RyanYoung25/FaceTrackingWaistDemo
https://github.com/RyanYoung25/TwoArmSoundDemo
https://github.com/RyanYoung25/BiotacArmDemo

Each demo has a README with installation instructions which includes specifying them in the ros package path. 
Some demos require other dependencies, see their README for more information. 


******************
**   STOPPING   **
******************

To stop MAESTOR:
	maestor kill

****************************
*****  DEFAULT NAMING  *****
****************************

Joints:

WST		Waist
NKY		Neck Yaw
NK1		Neck Tilt 1
NK2		Neck Tilt 2
LSP		Left Shoulder Pitch
LSR		Left Shoulder Roll
LSY		Left Shoulder Yaw
LEP		Left Elbow Pitch
LWY		Left Wrist Yaw
LWP		Left Wrist Pitch
RSP		Right Shoulder Pitch
RSR		Right Shoulder Roll
RSY		Right Shoulder Yaw
REP		Right Elbow Pitch
RWY		Right Wrist Yaw
RWP		Right Wrist Pitch
LHY		Left Hip Yaw
LHR		Left Hip Roll
LHP		Left Hip Pitch
LKP		Left Knee Pitch
LAP		Left Ankle Pitch
LAR		Left Ankle Roll
RHY		Right Hip Yaw
RHR		Right Hip Roll
RHP		Right Hip Pitch
RKP		Right Knee Pitch
RAP		Right Ankle Pitch
RAR		Right Ankle Roll
RF1		Right Finger 1
RF2		Right Finger 2
RF3		Right Finger 3
RF4		Right Finger 4
RF5		Right Finger 5
LF1		Left Finger 1
LF2		Left Finger 2
LF3		Left Finger 3
LF4		Left Finger 4
LF5		Left Finger 5

Sensors:

IMU		Body IMU
LAI		Left Ankle IMU
RAI		Right Ankle IMU

LAT		Left Ankle Force Torque
RAT		Right Ankle Force Torque
LWT		Left Wrist Force Torque
RWT		Right Wrist Force Torque

These names can be changed by aliasing them during runtime. See the Initialization section for details.


****************************************************
************  RESPONSE CHARACTERISTICS  ************
****************************************************

Maestor operates by default at an update cycle rate of 200 Hz. At these speeds, Maestor is able to push
joint positions down and receive the updated position back with an average latency of 10 ms. Currently,
it is questionable whether or not this response time is adequate for feedback loops. If the application
merely requires joint references, rather than a feedback loop, the latency for commands sent to the 
robot is less than 1 ms on average. Test statistics are located in /maestor/test/timing, though they
may be slightly cryptic to decipher.


****************************************************
*************          SUPPORT          ************
****************************************************

Contact us with any questions you have:

Ryan Young: rdy29@drexel.edu
Jacky Speck: jas522@drexel.edu
Eric Rock: igm@drexel.edu

