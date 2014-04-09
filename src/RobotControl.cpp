/*
Copyright (c) 2013, Drexel University, iSchool, Applied Informatics Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "RobotControl.h"

RobotControl::RobotControl(){

	RUN_TYPE = HARDWARE;

    commandChannel = CommandChannel::instance();
    referenceChannel = ReferenceChannel::instance();
    stateChannel = StateChannel::instance();
    if(RUN_TYPE == SIMULATION){
        simChannels = SimChannels::instance();
    }

    //ALL OF THESE NEED TO BE IMPLEMENTED AS SERVICES 
    /*

    this->addOperation("initRobot", & RobotControl::initRobot, this, RTT::OwnThread)
            .doc("Initialize a robot")
            .arg("Path", "The path to the XML robot representation");

    this->addOperation("setProperty", &RobotControl::set, this, RTT::OwnThread)
            .doc("Set a property of a robot subsystem")
            .arg("Name", "The name of the subsystem to set properties for.")
            .arg("Property", "The name of the property to change. See README for a list of properties and expected values.")
            .arg("Value", "The value to set the property to.");

    this->addOperation("setProperties", &RobotControl::setProperties, this, RTT::OwnThread)
            .doc("Set multiple properties of a robot subsystem.")
            .arg("Names", "The names of the subsystems to set properties for.")
            .arg("Properties", "The names of the properties to change. See README for a list of mutable properties.")
            .arg("Values", "The values to set these properties to.");

    this->addOperation("getProperty", &RobotControl::get, this, RTT::OwnThread)
            .doc("Get the value of a property of a robot subsystem.")
            .arg("Name", "The name of the subsystem to read values from.")
            .arg("Property", "The type of value to read. See README for a list of properties and expected values.");

    this->addOperation("getProperties", &RobotControl::getProperties, this, RTT::OwnThread)
            .doc("Get the value of a list of properties of a robot subsystem.")
            .arg("Name", "The name of the subsystem to read values from.")
            .arg("Property", "The list of values to read. See README for a list of properties and expected values.");

    this->addOperation("command", &RobotControl::command, this, RTT::OwnThread)
            .doc("Send a command to the robot.")
            .arg("Name", "The name of the command to send. See README for command list and arguments.")
            .arg("Target", "The target of the command. Usually a joint name.");

    this->addOperation("setMode", &RobotControl::setMode, this, RTT::OwnThread)
            .doc("Modify the mode of operation of RobotControl.")
            .arg("Mode", "The mode to be modified. Not yet documented.")
            .arg("Value", "New value for mode to take on. Not yet documented.");

    this->addOperation("setAlias", &RobotControl::setAlias, this, RTT::OwnThread)
            .doc("Create an alternate name for a motor, property, sensor, or command.")
            .arg("Name", "The name of said entity currently recognized")
            .arg("Alias", "An alias for said name. The old name will not be overwritten. You can not have the same name for different entities.");

    this->addOperation("requiresMotion", &RobotControl::requiresMotion, this, RTT::OwnThread)
            .arg("Name", "The name of the motor to check for necessary motion on.");

    this->addOperation("setDelay", &RobotControl::setDelay, this, RTT::OwnThread)
            .arg("Microseconds", "Delay amount in microseconds.");

    this->addOperation("loadTrajectory", &RobotControl::loadTrajectory, this, RTT::OwnThread)
            .arg("Name", "The name of the trajectory to load.")
            .arg("path", "Path to the trajectory to load.")
            .arg("read", "Boolean - True if reading a trajectory file, false if recording motion.");

    this->addOperation("ignoreFrom", &RobotControl::ignoreFrom, this, RTT::OwnThread)
            .arg("Name", "The name of the trajectory.")
            .arg("Col", "The name of the column to ignore.");

    this->addOperation("ignoreAllFrom", &RobotControl::ignoreAllFrom, this, RTT::OwnThread)
            .arg("Name", "The name of the trajectory.");

    this->addOperation("unignoreFrom", &RobotControl::unignoreFrom, this, RTT::OwnThread)
            .arg("Name", "The name of the trajectory.")
            .arg("Col", "The name of the column to unignore.");

    this->addOperation("unignoreAllFrom", &RobotControl::unignoreAllFrom, this, RTT::OwnThread)
            .arg("Name", "The name of the trajectory.");

    this->addOperation("setTrigger", &RobotControl::setTrigger, this, RTT::OwnThread)
            .arg("Name", "The name of the trajectory to set a trigger for.")
            .arg("frame", "The frame on which the trigger will fire.")
            .arg("target", "The trajectory to load upon firing");

    this->addOperation("extendTrajectory", &RobotControl::extendTrajectory, this, RTT::OwnThread)
            .arg("Name", "The name of the trajectory to load.")
            .arg("path", "Path to the trajectory to load.");

    this->addOperation("startTrajectory", &RobotControl::startTrajectory, this, RTT::OwnThread)
            .arg("Name", "The name of the trajectory to load.");

    this->addOperation("stopTrajectory", &RobotControl::stopTrajectory, this, RTT::OwnThread)
            .arg("Name", "The name of the trajectory to stop.");

    */

    this->written = 0;
    this->printNow = false;
    this->enableControl = false;
    this->delay = 0;
    this->state = NULL;
    this->interpolation = true;    //Interpret all commands as a final destination with given velocity.
    this->override = true;        //Force homing before allowing enabling. (currently disabled)

    commands["Enable"] = ENABLE;
    commands["EnableAll"] = ENABLEALL;
    commands["Disable"] = DISABLE;
    commands["DisableAll"] = DISABLEALL;
    commands["ResetJoint"] = RESET;
    commands["ResetAll"] = RESETALL;
    commands["Home"] = HOME;
    commands["HomeAll"] = HOMEALL;
    commands["InitializeSensors"] = INITSENSORS;
    commands["Update"] = UPDATE;
    commands["Zero"] = ZERO;
    commands["ZeroAll"] = ZEROALL;

    ostringstream logfile;
    logfile << LOG_PATH << "RobotControl.log";
    tempOutput.open(logfile.str().c_str());
   //vector<string> paths = getGestureScripts(CONFIG_PATH);
    /*
    for (int i = 0; i < paths.size(); i++){
        //cout << "Adding gestures from path: " << paths[i] << endl;
        this->getProvider<Scripting>("scripting")->loadPrograms(paths[i]);
    }
    */
    power = new PowerControlBoard();

    frames = 0;
    trajStarted = false;
}
  
RobotControl::~RobotControl(){
    delete state;
    delete power;
}

void RobotControl::updateHook(){
   /* commHandler->update();

    if (commHandler->isNew(1)){
        MaestroCommand message = commHandler->getPyMessage();
        if(message.command.compare("initRobot") == 0)
            initRobot("");

        if (state == NULL)
            return;
        this->handleMessage(message);
    }*/
    if (state == NULL)
        return;

    if(RUN_TYPE == SIMULATION){
        simChannels->load();
    }
    referenceChannel->load();
    stateChannel->load();


    trajStarted = trajectories.hasRunning();

    Boards boards = state->getBoards();
    Motors motors = state->getMotorMap();
    Trajectory* traj = NULL;
    HuboMotor* motor = NULL;


    if (!boards.empty()) {
        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;
            traj = trajectories.inRunning(motor->getName());

            if (motor->isEnabled()){
                double pos = 0;

                if (motor->getMode() == HUBO_REF_MODE_COMPLIANT){
                    // Compliance
                    motor->setInterStep(motor->getPosition());
                    motor->setGoalPosition(motor->getPosition());
                    pos = motor->getPosition();

                } else if (trajStarted && traj){
                    // Trajectory Playback
                    pos = motor->getGoalPosition();
                    motor->setMode(HUBO_REF_MODE_REF);
                    if (!traj->nextPosition(motor->getName(), pos) && !traj->hasNext()){
                        cout << "Reading of trajectory positions has terminated." << endl << "> ";
                        cout.flush(); // Fixing flushing issue with the deployer, maybe....
                        trajectories.stopTrajectory(traj);
                        trajStarted = trajectories.hasRunning();
                    }
                    motor->setGoalPosition(pos);
                    motor->setInterStep(pos);

                } else if (interpolation){
                    if (motor->requiresMotion())
                        pos = motor->interpolate();
                    else
                        pos = motor->getGoalPosition();
                    power->addMotionPower(motor->getName(), (1/200));  //TODO: Get the period
                    motor->setMode(HUBO_REF_MODE_REF_FILTER);
                } else {
                    pos = motor->getGoalPosition();
                }
                referenceChannel->setReference(motor->getName(), pos, motor->getMode());
                string key(WRITE_KEY);
                traj = trajectories.get(key);
                if (trajStarted && traj){
                    double currPos = get(motor->getName(), "position");

                    if (!traj->nextPosition(motor->getName(), currPos)){
                        cout << "Writing of trajectory positions has terminated." << endl << "> ";
                        cout.flush(); // Fixing flushing issue with the deployer, maybe....
                        trajectories.stopTrajectory(traj);
                        trajStarted = trajectories.hasRunning();
                    }
                }

                while (!trajectories.getCurrentTriggers().empty()){
                    startTrajectory(trajectories.getCurrentTriggers().front());
                    trajectories.getCurrentTriggers().pop();
                }
            }
        }
        if (trajStarted)
            trajectories.advanceFrame();
    }

    power->addMotionPower("IDLE", (1/200)); //TODO: get the period


    //Write out a message if we have one
    referenceChannel->update();

    usleep(delay);
}

//TODO: Scan directory for gestures, directory defined by config
/*vector<string> RobotControl::getGestureScripts(string path){
    vector<string> files;

    ifstream is;
    is.open(path.c_str());
    string temp;
    if (is.is_open()){
        do {
            getline(is, temp, '\n');
        } while (temp.compare("Scripts:") != 0);
        do {
            getline(is, temp, '\n');
            if (temp.compare("") != 0) files.push_back(temp);
        } while (!is.eof());
        is.close();
    } else
        cout << "Error. Config file nonexistent. Aborting." << endl;

    return files;
}*/

void RobotControl::setSimType(){
    RUN_TYPE = SIMULATION;
}

bool RobotControl::loadTrajectory(string name, string path, bool read){
    return trajectories.loadTrajectory(name, path, read);
}

bool RobotControl::ignoreFrom(string name, string col){
    return trajectories.ignoreFrom(name, col);
}

bool RobotControl::ignoreAllFrom(string name){
    return trajectories.ignoreAllFrom(name);
}

bool RobotControl::unignoreFrom(string name, string col){
    return trajectories.unignoreFrom(name, col);
}

bool RobotControl::unignoreAllFrom(string name){
    return trajectories.unignoreAllFrom(name);
}

bool RobotControl::setTrigger(string name, int frame, string target){
    return trajectories.setTrigger(name, frame, target);
}

bool RobotControl::extendTrajectory(string name, string path){
    return trajectories.extendTrajectory(name, path);
}

void RobotControl::startTrajectory(string name){
    Trajectory* traj = NULL;

    traj = trajectories.get(name);
    if (traj == NULL){
        std::cout << "No trajectory with name " << name << " loaded." << endl;
        return;
    }

    Motors motors = state->getMotorMap();
    if (traj->read_only()) {

        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            HuboMotor* motor = it->second;

            if (traj->contains(motor->getName())){
                if (!motor->isEnabled()){
                    cout << "Cannot start trajectory: references disabled motor " << motor->getName() << endl;
                    return;
                }
                double pos = get(motor->getName(), "position");
                if (fabs(pos - traj->startPosition(motor->getName())) > .1){
                    cout << "Cannot start trajectory: motor " << motor->getName() << " should be at " << traj->startPosition(motor->getName()) << endl;
                    return;
                }
            }
        }
    } else {
        Header header;

        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            HuboMotor* motor = it->second;
            if (motor->isEnabled())
                header.push_back(motor->getName());

        }
        traj->setHeader(header);
    }

    trajectories.startTrajectory(name);
}

void RobotControl::stopTrajectory(string name){
    trajectories.stopTrajectory(name);
    trajStarted = trajectories.hasRunning();
}

string RobotControl::getDefaultInitPath(string path){
    ifstream is;
    is.open(path.c_str());
    string temp;
    if (is.is_open()){
        do {
            getline(is, temp, '\n');
        } while (temp.compare("Init File:") != 0);

        getline(is, temp, '\n');
        is.close();
    } else
        cout << "Error. Config file nonexistent. Aborting." << endl;

    return temp;
}

void RobotControl::initRobot(string path){
    this->state = new HuboState();
    if (strcmp(path.c_str(), "") == 0)
    {
        path = getDefaultInitPath(CONFIG_PATH);
    }
    //@TODO: Check for file existence before initializing.
    this->state->initHuboWithDefaults(path, 200);  //TODO: get the period

    if (this->state == NULL)
    {
        std::cout << "Error. Initializing robot failed. Robot state is null." << std::endl;
    }

}

//GET RID OF BECAUSE EVERYTHING WILL BE A SERVICES
/*
void RobotControl::handleMessage(MaestroCommand message) {

	string joint = message.joint;
	string command = message.command;
	string target = message.target;
	string value = message.value;
    uint32_t id = message.id; 
	if(command.compare("initRobot") == 0) {
		initRobot("");
	}

	if (joint.compare("") == 0) {

		if (command.compare("initRobot") == 0)
			initRobot("");
        else if (command.compare("SetMode") == 0){

            bool boolValue = true;
            if(target.compare("false") == 0) {
                boolValue = false;
            }
            setMode(value, boolValue);
            cout << "Was handled correctly" << endl;
        }
		else
			this->command(command, target);

	} else if (command.compare("Get") == 0) {
		double value = get(joint, target);

		MaestroMessage newMessage;
		newMessage.joint = joint;
		newMessage.property = target;
		newMessage.value = value;
        newMessage.id = id;
		this->messageDownPort->write(newMessage);	
	} else if (command.compare("Check") == 0) {
		bool value = requiresMotion(joint);

		MaestroMessage newMessage;
		newMessage.joint = joint;
		newMessage.property = target;
		newMessage.value = value;
        newMessage.id = id;
		this->messageDownPort->write(newMessage);
	} else
		setProperties(joint, command, value);
}
*/

void RobotControl::set(string name, string property, double value){
    Motors motors = state->getMotorMap();

    if (motors.count(name) == 0){
        cout << "Error. Motor with name " << name << " is not on record. Aborting." << endl;
        return;
    }
    HuboMotor* motor = motors[name];

    Properties properties = state->getPropertyMap();
    if (properties.count(property) == 0){
        cout << "Error. No property with name " << property << " registered. Aborting." << endl;
        return;
    }

    switch (properties[property]){
    case POSITION:
        if (printNow) cout << "Setting position of motor " << name << " to " << value << " ." << endl;
        //power->addMotionPower(name, motor->getGoalPosition(), value);
        motor->setGoalPosition(value);
        break;
    case VELOCITY:
        if (printNow) cout << "Setting velocity of motor " << name << " to " << value << " ." << endl;
        motor->setInterVelocity(value);
        if (!interpolation) {
            cout << "Warning. RobotControl is not currently handling interpolation. " << endl <<
                    "Automatically enabling interpolation for velocity control." << endl << "> ";
            cout.flush();
            setMode("Interpolation",true);
        }
        break;
    case MOTION_TYPE:
        switch ((int)value) {
        case HUBO_REF_MODE_REF:
            if (printNow) cout << "Setting mode to direct reference control for motor " << motor->getName() << endl;
            motor->setMode(HUBO_REF_MODE_REF);
            break;
        case HUBO_REF_MODE_REF_FILTER:
            if (printNow) cout << "Setting mode to filtered reference control for motor " << motor->getName() << endl;
            motor->setMode(HUBO_REF_MODE_REF_FILTER);
            break;
        case HUBO_REF_MODE_COMPLIANT:
            if (printNow) cout << "Setting mode to compliant control for motor " << motor->getName() << endl;
            motor->setMode(HUBO_REF_MODE_COMPLIANT);
            break;
        case HUBO_REF_MODE_ENC_FILTER:
            if (printNow) cout << "Setting mode to encoder filtered control for motor " << motor->getName() << endl;
            motor->setMode(HUBO_REF_MODE_ENC_FILTER);
            break;
        // case HUBO_REF_MODE_REFX:
        //     if (printNow) cout << "Setting mode to differential reference control for motor " << motor->getName() << endl;
        //     motor->setMode(HUBO_REF_MODE_REFX);
        //     break;
        default:
            cout << "Motion type " << value << " not recognized." << endl;
            return;
        }
        break;
    default:
        cout << "Motor with name " << name << " has no mutable property named " << property << " ." << endl;
        return;
    }
}

void RobotControl::setProperties(string names, string properties, string values){
    vector<string> namesList = splitFields(names);
    vector<string> propertiesList = splitFields(properties);
    vector<string> valuesList = splitFields(values);
    if (namesList.size() != propertiesList.size()
            || namesList.size() != valuesList.size()
            || propertiesList.size() != valuesList.size()){
        cout << "Error! Size of entered fields not consistent. Aborting." << endl;
        return;
    }

    for (int i = 0; i < namesList.size(); i++){
        istringstream data(valuesList[i]);
        double value = 0;
        data >> value;

        set(namesList[i], propertiesList[i], value);
    }
}

double RobotControl::get(string name, string property){
    Motors motors = state->getMotorMap();
    FTSensors ftSensors = state->getFTSensorMap();
    IMUSensors imuSensors = state->getIMUSensorMap();
    Properties properties = state->getPropertyMap();

    double result = 0;

    if (properties.count(property) == 0){
        cout << "Error. No property with name " << property << " registered. Aborting." << endl;
        return 0;
    }

    if (motors.count(name) == 1){

        if (!stateChannel->getMotorProperty(name, properties[property], result)){
            cout << "Error requesting property " << property << " from state channel." << endl;
            return 0;
        }
        return result;

    } else if (ftSensors.count(name) == 1){

        if (!stateChannel->getFTProperty(name, properties[property], result)){
            cout << "Error requesting property " << property << " from state channel." << endl;
            return 0;
        }
        return result;

    } else if (imuSensors.count(name) == 1){

        if (!stateChannel->getIMUProperty(name, properties[property], result)){
            std::cout << "Error requesting property " << property << " from state channel." << std::endl;
            return 0;
        }
        return result;

    } else if (name.compare("PWR") == 0){
        return power->getTotalPowerUsed();
    } else {
        cout << "Error. Readable Object with name " << name << " is not on record. Aborting." << endl;
        return 0;
    }
}

string RobotControl::getProperties(string name, string properties) {
    vector<string> propertyList = splitFields(properties);
    ostringstream values;
    for (int i = 0; i < propertyList.size(); ++i)
    {
        double tmp = get(name, propertyList[i]);
        values << tmp;
        if(i + 1 != propertyList.size())
        {
            values << ", ";
        }
    }
    return values.str();
}

void RobotControl::command(string name, string target){
    Motors motors = state->getMotorMap();

    HuboMotor* motor;

    //only for normalization
    double pos;
    string joint;

    if (commands.count(name) == 0){
        cout << "Error. No command with name " << name << " is defined for RobotControl. Aborting." << endl << "> ";
        cout.flush();
        return;
    }

    switch (commands[name]){
    case ENABLE:
        if (motors.count(target) == 0){
            cout << "Error. Motor with name " << target << " is not on record. Aborting." << endl << "> ";
            cout.flush();
            return;
        }

        if (!this->commandChannel->enable(target)){
            cout << "Enable command failed. Aborting." << endl;
            return;
        }

        motor = motors[target];
        motor->setGoalPosition(get(motor->getName(), "position"));

        motor->setEnabled(true);
        /*
        if (RUN_TYPE == HARDWARE && !motor->isHomed()){
            cout << "Warning! Motor " << target << " has not yet been homed. Skipping enabling of this motor." << endl;
            return;
        }
        if (RUN_TYPE == HARDWARE)
            motor->setGoalPosition(motor->getPosition());
            */

        break;
    case ENABLEALL:

        if (!this->commandChannel->enable("all")){
            cout << "Enable command failed. Aborting." << endl;
            return;
        }

        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;

            /*
            if (RUN_TYPE == HARDWARE && !motor->isHomed()){
                cout << "Warning! Motor " << motor->getName() << " has not yet been homed. Skipping enabling of this motor." << endl;
                continue;
            }
            if (RUN_TYPE == HARDWARE)
                motor->setGoalPosition(motor->getPosition());
            */
            motor->setGoalPosition(get(motor->getName(), "position"));
            motor->setEnabled(true);
        }

        break;
    case DISABLE:
        if (motors.count(target) == 0){
            cout << "Error. Motor with name " << target << " is not on record. Aborting.";
            return;
        }

        if (!this->commandChannel->disable(target)){
            cout << "Disable command failed. Aborting." << endl;
            return;
        }

        motor = motors[target];
        motor->setEnabled(false);
        break;
    case DISABLEALL:
        if (!this->commandChannel->disable("all")){
            cout << "Disable command failed. Aborting." << endl;
            return;
        }


        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;
            motor->setEnabled(false);
        }
        break;
    case RESET:
        if (motors.count(target) == 0){
            cout << "Error. Motor with name " << target << " is not on record. Aborting.";
            return;
        }

        if (!this->commandChannel->reset(target)){
            cout << "Reset command failed. Aborting." << endl;
            return;
        }
        break;
    case RESETALL:
        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;
            command("ResetJoint", motor->getName());
        }
        return;
    case HOME:
        if (motors.count(target) == 0){
            cout << "Error. Motor with name " << target << " is not on record. Aborting.";
            return;
        }

        if (!this->commandChannel->home(target)){
            cout << "Homing command failed. Aborting." << endl;
            return;
        }

        motor = motors[target];
        set(target, "position", 0);
        break;
    case HOMEALL:
        if (!this->commandChannel->home("all")){
            cout << "Homing command failed. Aborting." << endl;
            return;
        }

        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;
            set(motor->getName(), "position", 0);
        }
        break;
        //TODO: Find a way to pause for a length of time here.
    case INITSENSORS:

        if (!this->commandChannel->initializeSensors()){
            cout << "Initialize Sensors command failed. Aborting." << endl;
            return;
        }
        break;
    case ZERO:
		if (motors.count(target) == 0){
			std::cout << "Error. Motor with name " << target << " is not on record. Aborting.";
			return;
		}
		set(target, "position", 0);
		break;
	case ZEROALL:
	    for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;
            set(motor->getName(), "position", 0);
        }
		break;
	case UPDATE:
        //updateState();
        break;
    }
}

void RobotControl::setMode(string mode, bool value){
    Motors motors = state->getMotorMap();
    HuboMotor* motor = NULL;

    if (mode.compare("Interpolation") == 0){
        if (printNow) cout << "Setting interpolation " << (value ? "on." : "off.") << endl;

        //If we are switching to interpolation, the internal step of each motor must be updated.
        if (value && !this->state->getBoards().empty()){

            for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
                motor = it->second;
                if (motor->isEnabled()){
                    double pos = motor->getInterStep();
                    if (!stateChannel->getMotorProperty(motor->getName(), POSITION, pos))
                        continue;
                    motor->setInterStep(pos);

                }
            }
        }
        interpolation = value;
    } else {
        cout << "RobotControl does not have a mutable mode with name " << mode << "." << endl;
    }
}

bool RobotControl::setAlias(string name, string alias){
    Motors motors = state->getMotorMap();
    FTSensors ftSensors = state->getFTSensorMap();
    IMUSensors imuSensors = state->getIMUSensorMap();
    Properties properties = state->getPropertyMap();

    int entries = 0;

    entries += motors.count(alias);
    entries += ftSensors.count(alias);
    entries += imuSensors.count(alias);
    entries += properties.count(alias);
    entries += commands.count(alias);

    if (entries > 0){
        cout << "There already exists an entity named " << alias << " in RobotControl." << cout;
        return false;
    } else if (motors.count(name) == 1)
        motors[alias] = motors[name];
    else if (ftSensors.count(name) == 1)
        ftSensors[alias] = ftSensors[name];
    else if (imuSensors.count(name) == 1)
        imuSensors[alias] = imuSensors[name];
    else if (properties.count(name) == 1)
        properties[alias] = properties[name];
    else if (commands.count(name) == 1)
        commands[alias] = commands[name];

    return true;
}

vector<string> RobotControl::splitFields(string input){
    vector<string> output;
    int whitespaceType = 0;
    if (input.find(' ') != string::npos) whitespaceType += 1;
    if (input.find('\t') != string::npos) whitespaceType += 2;
    if (input.find('\n') != string::npos) whitespaceType += 4;

    char whitespace;
    switch (whitespaceType){
    case 1:
        whitespace = ' ';
        break;
    case 2:
        whitespace = '\t';
        break;
    case 4:
        whitespace = '\n';
        break;
    default:
        output.push_back(input);
        return output;
    }
    string field;
    int pos = 0;

    do {
        pos = input.find(whitespace);
        field = input.substr(0, pos);
        output.push_back(field);
        input = input.substr(pos + 1, input.length() - pos - 1);
    } while (input.find(whitespace) != string::npos);
    output.push_back(input);
    return output;
}

void RobotControl::setDelay(int us){
    this->delay = us;
}

bool RobotControl::requiresMotion(string name){
    Motors motors = state->getMotorMap();
    if (motors.count(name) == 0){
        cout << "Error. Motor with name " << name << " is not on record. Aborting." << endl;
        return false;
    }
    return motors[name]->requiresMotion();
}

/*
void RobotControl::runGesture(string name){
    boost::shared_ptr<Scripting> scripting = this->getProvider<Scripting>("scripting");
    scripting->startProgram(name);
    if (!scripting->isProgramRunning(name))
        cout << "Error. Program not running." << endl;
    if (scripting->inProgramError(name))
        cout << "Error. Program has encountered an error. " << endl;
}
*/

