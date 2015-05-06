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

/**
 * Creates the Robot Control object. This initializes a lot of the important variables but it does not
 * fully inititalize MAESTOR.
 */
RobotControl::RobotControl(){

   	RUN_TYPE = HARDWARE;

    commandChannel = CommandChannel::instance();
    referenceChannel = ReferenceChannel::instance();
    stateChannel = StateChannel::instance();
    if(RUN_TYPE == SIMULATION){
        simChannels = SimChannels::instance();
    }

    this->state = HuboState::instance();


    this->written = 0;
    this->printNow = false;
    this->enableControl = false;
    this->delay = 0;
    this->interpolation = true;    //Interpret all commands as a final destination with given velocity.
    this->override = true;        //Force homing before allowing enabling. (currently disabled)
    this->balanceOn = false;
    
    Names::initPropertyMap();
    Names::initCommandMap();

    //ostringstream logfile;
    //logfile << "RobotControl.log";
    //tempOutput.open(logfile.str().c_str());
    power = new PowerControlBoard();

    balancer = new BalanceController();
    cout << "Made the balancer" << endl;

    frames = 0;
    trajStarted = false;
}

/**
 * Cleans up the Robot Control object.
 */
RobotControl::~RobotControl(){
    delete power;
    delete balancer;
}


/**
 * UpdateHook is the main function that gets called at the update frequency. This is where all of the 
 * state changes are applied. 
 */
void RobotControl::updateHook(){
    if (state == NULL)
        return;
    if(RUN_TYPE == SIMULATION){
        simChannels->load();
    }
    referenceChannel->load();
    stateChannel->load();

    trajStarted = trajectories.hasRunning();

    Components components = state->getComponents();
    Trajectory* traj = NULL;
    RobotComponent* component = NULL;

    if (!components.empty()) {
        if(balanceOn){
            balancer->Balance();
        }
        for (int i = 0; i < components.size(); i++){
            component = components[i];
            double enabled;
            if (!component->get(ENABLED, enabled))
                continue;
            traj = trajectories.inRunning(component->getName());

            if ((bool)enabled){
                double pos = 0;

                double mode = HUBO_REF_MODE_REF_FILTER;
                component->get(MOTION_TYPE, mode);
                if ((hubo_mode_type_t)mode == HUBO_REF_MODE_COMPLIANT){
                    // Compliance
                    component->get(POSITION, pos);
                    component->set(INTERPOLATION_STEP, pos);
                    component->set(GOAL, pos);
                } else if (trajStarted && traj){
                    // Trajectory Playback
                    component->get(GOAL, pos);
                    component->set(MOTION_TYPE, HUBO_REF_MODE_REF);
                    if (!traj->nextPosition(component->getName(), pos) && !traj->hasNext()){
                        cout << "Reading of trajectory positions has terminated." << endl << "> ";
                        cout.flush(); 
                        trajectories.stopTrajectory(traj);
                        trajStarted = trajectories.hasRunning();
                    }
                    component->set(GOAL, pos);
                    component->get(INTERPOLATION_STEP, pos);

                } else if (interpolation){
                    component->get(INTERPOLATION_STEP, pos);
                    power->addMotionPower(component->getName(), 1/PERIOD); 
                    component->set(MOTION_TYPE, HUBO_REF_MODE_REF);
                } else {
                    component->get(GOAL, pos);
                }
                component->get(MOTION_TYPE, mode);
                //tempOutput << component->getName() << ": " << pos << endl;
                referenceChannel->setReference(component->getName(), pos, (hubo_mode_type_t)mode);
                string key(WRITE_KEY);
                traj = trajectories.get(key);
                if (trajStarted && traj){
                    double currPos = 0;
                    component->get(POSITION, currPos);

                    if (traj->contains(component->getName()) && !traj->nextPosition(component->getName(), currPos)){
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

    power->addMotionPower("IDLE", PERIOD); 


    //Write out a message if we have one
    referenceChannel->update();
}

/**
 * Sets the run type to simulation. Used when MAESTOR is operated in simulation mode
 */
void RobotControl::setSimType(){
    simChannels = SimChannels::instance();
    RUN_TYPE = SIMULATION;
}

/**
 * Loads a trajectory into maestor. 
 * @param  name The name of the trajectory to load into maestor
 * @param  path The path to the trajectory to be loaded
 * @param  read When read is true we read from the trajectory, false we write to it
 * @return      true on succcess
 */
bool RobotControl::loadTrajectory(string name, string path, bool read){
    return trajectories.loadTrajectory(name, path, read);
}

/**
 * Ignore the column from the trajectory
 * @param  name The name of the trajcetory
 * @param  col  the name of the column to ignore
 * @return      true on success
 */
bool RobotControl::ignoreFrom(string name, string col){
    return trajectories.ignoreFrom(name, col);
}

/**
 * Ignore all of the columns from the trajectory 
 * @param  name name of the trajectory
 * @return      true on success
 */
bool RobotControl::ignoreAllFrom(string name){
    return trajectories.ignoreAllFrom(name);
}

/**
 * unignore the column from the trajectory
 * @param  name name of the trajectory 
 * @param  col  name of the column
 * @return      true on success
 */
bool RobotControl::unignoreFrom(string name, string col){
    return trajectories.unignoreFrom(name, col);
}

/**
 * unignore all the columns from the trajectory
 * @param  name name of the traj
 * @return      true on success
 */
bool RobotControl::unignoreAllFrom(string name){
    return trajectories.unignoreAllFrom(name);
}

/**
 * Set the trigger to start another trajectory at the target of the first one. 
 * @param  name   Name of the first trajectory.
 * @param  frame  The frame to start the next trajectory on
 * @param  target The next trajectory to play
 * @return        True on success
 */
bool RobotControl::setTrigger(string name, int frame, string target){
    return trajectories.setTrigger(name, frame, target);
}

/**
 * Extend the trajectory that is all ready loaded by the trajectory that 
 * is located at the path specified. 
 * @param  name Name of the loaded trajectory
 * @param  path The path to the trajectory that you want at the end of the first
 * @return      True on success
 */
bool RobotControl::extendTrajectory(string name, string path){
    return trajectories.extendTrajectory(name, path);
}

/**
 * Start a loaded trajectory. 
 * @param name The name of the trajectory to start. 
 */
void RobotControl::startTrajectory(string name){
    Trajectory* traj = NULL;

    traj = trajectories.get(name);
    if (traj == NULL){
        std::cout << "No trajectory with name " << name << " loaded." << endl;
        return;
    }

    Components components = state->getComponents();
    if (traj->read_only()) {

        for (Components::iterator it = components.begin(); it != components.end(); it++){
            RobotComponent* component = *it;
            string name = component->getName();

            if (traj->contains(name)){
                double enabled;
                if (!component->get(ENABLED, enabled))
                    continue;

                if (!(bool)enabled){
                    cout << "Cannot start trajectory: references disabled motor " << name << endl;
                    return;
                }
                double pos;
                component->get(POSITION, pos);
                if (fabs(pos - traj->startPosition(name)) > .1){
                    cout << "Cannot start trajectory: motor " << name << " should be at " << traj->startPosition(name) << endl;
                    return;
                }
            }
        }
    } else {
        Header header;

        for (Motors::const_iterator it = state->getMotors().begin(); it != state->getMotors().end(); it++){
            HuboMotor* component = *it;

            double enabled;
            if (!component->get(ENABLED, enabled))
                continue;

            if ((bool)enabled)
                header.push_back(component->getName());

        }
        traj->setHeader(header);
    }

    trajectories.startTrajectory(name);
}

/**
 * Stop a trajectory that is running. 
 * @param name Name of the trajectory to stop. 
 */
void RobotControl::stopTrajectory(string name){
    trajectories.stopTrajectory(name);
    trajStarted = trajectories.hasRunning();
}

/**
 * Get the initial config file path
 * @param  path the path to the file containing the path to the config file
 * @return      returns the path to the config file
 */
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

/**
 * Initialize the MAESTOR system by creating all of the robot components that are loaded from 
 * an XML configuration file. Empty string defaults to the default config path. 
 * @param path The path to the xml config file
 */
void RobotControl::initRobot(string path){
    if (strcmp(path.c_str(), "") == 0)
    {
        path = getDefaultInitPath(CONFIG_PATH);
    }
    
    this->state->initHuboWithDefaults(path, 1/PERIOD);  
    balancer->initBalanceController(*(this->state));

    if (this->state == NULL)
    {
        std::cout << "Error. Initializing robot failed. Robot state is null." << std::endl;
    }

}

/**
 * This is the set properties method. It sets a components property to the value. 
 * @param name     The name of the robot component
 * @param property Name of the property to be set
 * @param value    Value to set to the property
 */
void RobotControl::set(string name, string property, double value){
    if (!state->nameExists(name)){
        cout << "Error. No component with name " << name << " registered. Aborting." << endl;
        return;
    }

    Properties properties = Names::getProps();

    if (properties.count(property) == 0){
        cout << "Error. No property with name " << property << " registered. Aborting." << endl;
        return;
    }

    if (!state->getComponent(name)->set(properties[property], value)){
        cout << "Error setting property " << property << " of component " << name << endl;
        return;
    }
}

/**
 * Set multiple properties on multiple robot components to multiple values. There must be an equal number
 * of robot components, properties, and values for the function to work properly. 
 * @param names      Space delimited string of robot component names
 * @param properties Space delimited string of properties
 * @param values     Space delimited string of values
 */
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

/**
 * Get the value of a property for a specific robot componenet
 * @param  name     Name of the robot component
 * @param  property Name of the property
 * @return          Value of the property for that robot component
 */
double RobotControl::get(string name, string property){
    
    double result = 0;

    if(name.compare("ZMP") == 0){
        if(property.compare("X") == 0){
            result = balancer->getZMP(0);
        }
        else if (property.compare("Y") == 0){
            result = balancer->getZMP(1);
        }
        else{
            cout << "Error getting property " << property << " of component " << name << endl;
            return 0;
        }
        return result;
    }


    if (!state->nameExists(name)){
        cout << "Error. No component with name " << name << " registered. Aborting." << endl;
        return 0;
    }

    Properties properties = Names::getProps();

    if (properties.count(property) == 0){
        cout << "Error. No property with name " << property << " registered. Aborting." << endl;
        return 0;
    }

    if (!state->getComponent(name)->get(properties[property], result)){
        cout << "Error getting property " << property << " of component " << name << endl;
        return 0;
    }

    return result;


}

/**
 * Get multiple properties for a given robot component. 
 * @param  name       Name of the robot component
 * @param  properties Space delimited string of properties
 * @return            A space delimited string of values for each property it found
 */
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

/**
 * Run the command that is passed in. If it has a joint target run it on that joint. 
 * @param name   Name of the command to run
 * @param target Optional Joint target
 */
void RobotControl::command(string name, string target){
    Commands commands = Names::getComms();
    RobotComponent* component = NULL;
    double temp;


    if (commands.count(name) == 0){
        cout << "Error. No command with name " << name << " is defined for RobotControl. Aborting." << endl << "> ";
        cout.flush();
        return;
    }

    switch (commands[name]){
    case ENABLE:
        if (!state->nameExists(target)){
            cout << "Error. Component with name " << target << " is not on record. Aborting." << endl << "> ";
            cout.flush();
            return;
        }

        if (!this->commandChannel->enable(target)){
            cout << "Enable command failed. Aborting." << endl;
            return;
        }

        component = state->getComponent(target);
        if (component == NULL){
            cout << "Error retrieving component with name " << target << endl;
            return;
        }

        double pos;
        component->get(POSITION, pos);
        component->set(GOAL, pos);

        component->set(ENABLED, true);

        break;
    case ENABLEALL:

        if (!this->commandChannel->enable("all")){
            cout << "Enable command failed. Aborting." << endl;
            return;
        }

        for (Motors::const_iterator it = state->getMotors().begin(); it != state->getMotors().end(); it++){
            component = *it;

            component->get(POSITION, temp);
            component->set(GOAL, temp);
            component->set(ENABLED, true);
        }

        break;
    case DISABLE:
        if (!state->nameExists(target)){
            cout << "Error. Component with name " << target << " is not on record. Aborting." << endl << "> ";
            cout.flush();
            return;
        }

        if (!this->commandChannel->disable(target)){
            cout << "Disable command failed. Aborting." << endl;
            return;
        }

        component = state->getComponent(target);
        if (component == NULL){
            cout << "Error retrieving component with name " << target << endl;
            return;
        }

        component->set(ENABLED, false);
        break;
    case DISABLEALL:
        if (!this->commandChannel->disable("all")){
            cout << "Disable command failed. Aborting." << endl;
            return;
        }

        for (Motors::const_iterator it = state->getMotors().begin(); it != state->getMotors().end(); it++)
            (*it)->set(ENABLED, false);

        break;
    case RESET:
        if (!state->nameExists(target)){
           cout << "Error. Component with name " << target << " is not on record. Aborting." << endl << "> ";
           cout.flush();
           return;
       }

        if (!this->commandChannel->reset(target)){
            cout << "Reset command failed. Aborting." << endl;
            return;
        }
        break;
    case RESETALL:
        for (Motors::const_iterator it = state->getMotors().begin(); it != state->getMotors().end(); it++)
            command("ResetJoint", (*it)->getName());

        return;
    case HOME:
        if (!state->nameExists(target)){
           cout << "Error. Component with name " << target << " is not on record. Aborting." << endl << "> ";
           cout.flush();
           return;
       }

        if (!this->commandChannel->home(target)){
            cout << "Homing command failed. Aborting." << endl;
            return;
        }

        component = state->getComponent(target);
        if (component == NULL){
            cout << "Error retrieving component with name " << target << endl;
            return;
        }
        component->set(GOAL, 0);
        break;
    case HOMEALL:
        if (!this->commandChannel->home("all")){
            cout << "Homing command failed. Aborting." << endl;
            return;
        }

        for (Motors::const_iterator it = state->getMotors().begin(); it != state->getMotors().end(); it++)
            (*it)->set(GOAL, 0);

        break;
    case INITSENSORS:

        if (!this->commandChannel->initializeSensors()){
            cout << "Initialize Sensors command failed. Aborting." << endl;
            return;
        }
        break;
    case ZERO:
        component = state->getComponent(target);
        if (component == NULL){
            cout << "Error retrieving component with name " << target << endl;
            return;
        }
        if (!component->get(ENABLED, temp)){
            cout << "Attempting to zero a non-motor component. Aborting" << endl;
            return;
        }
        component->set(GOAL, 0);
        break;
    case ZEROALL:
        for (Motors::const_iterator it = state->getMotors().begin(); it != state->getMotors().end(); it++)
            (*it)->set(GOAL, 0);
        break;
    case UPDATE:
        break;
    case BALANCEON:
        balanceOn = true;
        balancer->setBaseline();
        break;
    case BALANCEOFF:
        balanceOn = false;
        break;

    }
}

/**
 * Set the mode of MAESTOR. This can be used to turn interpolation off. This was deemed hazardous so it
 * was taken out of the ROS service list. Proceed with caution. 
 * @param mode  the only mode is interpolation
 * @param value on or off
 */
void RobotControl::setMode(string mode, bool value){
    Components components = state->getComponents();
    RobotComponent* component = NULL;
    HuboMotor* motor = NULL;

    if (mode.compare("Interpolation") == 0){
        if (printNow) cout << "Setting interpolation " << (value ? "on." : "off.") << endl;

        //If we are switching to interpolation, the internal step of each motor must be updated.
        if (value && !components.empty()){

            for (Components::iterator it = components.begin(); it != components.end(); it++){
                component = *it;

                double enabled;
                if (!component->get(ENABLED, enabled))
                    continue;

                if ((bool)enabled){
                    motor = static_cast<HuboMotor*> (component);

                    double pos = 0;
                    if (!motor->get(POSITION, pos))
                        continue;
                    motor->set(INTERPOLATION_STEP, pos);
                }
            }
        }
        interpolation = value;
    } else {
        cout << "RobotControl does not have a mutable mode with name " << mode << "." << endl;
    }
}

/**
 * Set the period for MAESTOR
 * @param period the period in seconds
 */
void RobotControl::setPeriod(double period){
    PERIOD = period;
}

/**
 * Set an alias for a robot component or command
 * @param  name  Name of the robot component or command
 * @param  alias alias to add
 * @return       True on success
 */
bool RobotControl::setAlias(string name, string alias){
    bool res = Names::setAlias(name, alias);
    if(res == true){
        return res;
    }
    return state->setAlias(name, alias);
}

/**
 * Convience method to split a string into it's fields
 * @param input The string to split
 * @return      A vector of each field
 */
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

/**
 * Set a delay
 * @param us The delay in microseconds
 */
void RobotControl::setDelay(int us){
    this->delay = us;
}

/**
 * Check if a joint is at its goal. If it is at its goal it does 
 * not require motion otherwise it does
 * @param  name Joint to check
 * @return      True if the joint is not at its goal False if it is. 
 */
bool RobotControl::requiresMotion(string name){
    RobotComponent* component = state->getComponent(name);
    if (component == NULL){
        cout << "Error retrieving component with name " << name << endl;
        return false;
    }
    double step, goal;
    if (!component->get(POSITION, step) || !component->get(GOAL, goal)){
        cout << "Error retrieving data from component " << name << endl;
        return false;
    }

    return fabs(step - goal) > .01;
}
