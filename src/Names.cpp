/**
 * All of the names of properties and commands that we use in MAESTOR
 * They are all consolidated in here so we don't have a lot of repetition 
 * and so we can use aliases 
 */
#include "Names.h"

/**
 * Initialize all of the properties
 */
void Names::initPropertyMap(){
    std::cout << "Called initPropertyMap" << std::endl;
    getProperties()["position"] = POSITION;
    getProperties()["goal"] = GOAL;
    getProperties()["speed"] = SPEED;
    getProperties()["inter_step"] = INTERPOLATION_STEP;
    getProperties()["velocity"] = VELOCITY;
    getProperties()["goal_time"] = GOAL_TIME;
    getProperties()["motion_type"] = MOTION_TYPE;
    getProperties()["temp"] = TEMPERATURE;
    getProperties()["homed"] = HOMED;
    getProperties()["zeroed"] = ZEROED;
    getProperties()["enabled"] = ENABLED;
    getProperties()["errored"] = ERRORED;
    getProperties()["jamError"] = JAM_ERROR;
    getProperties()["PWMSaturatedError"] = PWM_SATURATED_ERROR;
    getProperties()["bigError"] = BIG_ERROR;
    getProperties()["encoderError"] = ENC_ERROR;
    getProperties()["driveFaultError"] = DRIVE_FAULT_ERROR;
    getProperties()["posMinError"] = POS_MIN_ERROR;
    getProperties()["posMaxError"] = POS_MAX_ERROR;
    getProperties()["velocityError"] = VELOCITY_ERROR;
    getProperties()["accelerationError"] = ACCELERATION_ERROR;
    getProperties()["tempError"] = TEMP_ERROR;
    getProperties()["x_acc"] = X_ACCEL;
    getProperties()["y_acc"] = Y_ACCEL;
    getProperties()["z_acc"] = Z_ACCEL;
    getProperties()["x_rot"] = X_ROTAT;
    getProperties()["y_rot"] = Y_ROTAT;
    getProperties()["m_x"] = M_X;
    getProperties()["m_y"] = M_Y;
    getProperties()["f_z"] = F_Z;
    getProperties()["meta_value"] = META_VALUE;
    getProperties()["ready"] = READY;
}

/**
 * Initialize all of the commands
 */
void Names::initCommandMap(){
    getCommands()["Enable"] = ENABLE;
    getCommands()["EnableAll"] = ENABLEALL;
    getCommands()["Disable"] = DISABLE;
    getCommands()["DisableAll"] = DISABLEALL;
    getCommands()["ResetJoint"] = RESET;
    getCommands()["ResetAll"] = RESETALL;
    getCommands()["Home"] = HOME;
    getCommands()["HomeAll"] = HOMEALL;
    getCommands()["InitializeSensors"] = INITSENSORS;
    getCommands()["Update"] = UPDATE;
    getCommands()["Zero"] = ZERO;
    getCommands()["ZeroAll"] = ZEROALL;
    getCommands()["BalanceOn"] = BALANCEON;
    getCommands()["BalanceOff"] = BALANCEOFF;
}

/**
 * Set an alias for a command or property
 * @param  name  The original command or property
 * @param  alias The alias to replace it with 
 * @return       True on success
 */
bool Names::setAlias(string name, string alias){
    if (getProperties().count(name) == 1 && getProperties().count(alias) == 0) {
        getProperties()[alias] = getProperties()[name];
        return true;
    } else if (getCommands().count(name) == 1 && getProperties().count(alias) == 0) {
        getCommands()[alias] = getCommands()[name];
        return true;
    }
    return false;
}

/**
 * Get the string name of a property from it's enum
 * @param  property The enum version of the property
 * @return          The string representation of the property
 */
string Names::getName(PROPERTY &property){
    for (Properties::iterator it = getProperties().begin(); it != getProperties().end(); it++){
        if (it->second == property)
            return it->first;
    }
    return "NULL PROPERTY";
}

/**
 * Get the string name of a command from it's enum
 * @param  command The enum version of the command
 * @return          The string representation of the command
 */
string Names::getName(COMMAND &command){
    for (Commands::iterator it = getCommands().begin(); it != getCommands().end(); it++){
        if (it->second == command)
            return it->first;
    }
    return "NUlL COMMAND";
}
