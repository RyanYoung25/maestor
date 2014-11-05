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

/**
 * This is a balance controller object that handles hubo balancing. 
 * A lot of it was taken directly from RAINBOW code but some things 
 * needed to be changed or modifed to make it work. It's kinda sloppy
 * but it gets the job done. If I can I will try to clean it up in the future. 
 */
#include "BalanceController.h"

/**
 * Create a balance controller
 */
BalanceController::BalanceController(){
    dampingGain[0] = 0.2f;      dampingGain[3] = 0.2f;
    dampingGain[1] = 1.0f;      dampingGain[4] = 1.0f;
    dampingGain[2] = 0.4f;      dampingGain[5] = 0.5f;

    initialized = false;

    balanceComponents[0] = "RFX"; 
    balanceComponents[2] = "RFY"; 
    balanceComponents[1] = "RFZ"; 
    balanceComponents[3] = "LFX"; 
    balanceComponents[4] = "LFY"; 
    balanceComponents[5] = "LFZ"; 
    balanceComponents[6] = "RAT"; 
    balanceComponents[7] = "LAT"; 
    balanceComponents[8] = "IMU"; 

}

/**
 * Destructor
 */
BalanceController::~BalanceController(){}

/**
 * Turn try to Balance the robot. It calcluates the correct offsets 
 * for standing on two feet and balancing the robot on uneven surfaces. 
 */
void BalanceController::Balance(){
    getCurrentSupportPhase();
    ZMPcalculation();
    DSPControl();
    DampingControl();

    setOffset("RFX", ControlDSP[0][0]);
    setOffset("RFY", ControlDSP[0][1]);
    setOffset("LFX", ControlDSP[1][0]);
    setOffset("LFY", ControlDSP[1][1]);
    //Active balance attempt
    //If the joint does not require motion then we can set it's new goal as the current position plus 
    //the offset. 
    double RFx = BalanceController::get("RFX", "position"); 
    double RFy = BalanceController::get("RFY", "position");
    double LFx = BalanceController::get("LFX", "position");
    double LFy = BalanceController::get("LFY", "position");
    //rounded offsets and adjusted 
    double Rx = (-1 * floor(ControlDSP[0][0]*1000) / 1000) - BaseDSP[0][0]; 
    double Ry = (-1 * floor(ControlDSP[0][1]*1000) / 1000) - BaseDSP[0][1];
    double Lx = (-1 * floor(ControlDSP[1][0]*1000) / 1000) - BaseDSP[1][0];
    double Ly = (-1 * floor(ControlDSP[1][1]*1000) / 1000) - BaseDSP[1][1]; 
    // If too big go back
    if(fabs(Rx) > .015){
        Rx = ((Rx > 0)-(Rx < 0)) * 0.015; // check the sign and cap it
    }
    if(fabs(Ry) > .015){
        Ry = ((Ry > 0)-(Ry < 0)) * 0.015; // check the sign and cap it
    }
    if(fabs(Lx) > .015){
        Lx = ((Lx > 0)-(Lx < 0)) * 0.015; // check the sign and cap it
    }
    if(fabs(Ly) > .015){
        Ly = ((Ly > 0)-(Ly < 0)) * 0.015; // check the sign and cap it
    }
   

    // New position
    double Rxpos = RFx + Rx;   
    double Rypos = RFy + Ry;
    double Lxpos = LFx + Lx;
    double Lypos = LFy + Ly;

    if(!requiresMotion("RFX") && fabs(Rx) > .005){
        BalanceController::set("RFX", "position", Rxpos);
    }
    if(!requiresMotion("RFY") && fabs(Ry) > .005){
        BalanceController::set("RFY", "position", Rypos);
    }
    if(!requiresMotion("LFX") && fabs(Lx) > .005){
        BalanceController::set("LFX", "position", Lxpos);
    }
    if(!requiresMotion("LFY") && fabs(Ly) > .005){
        BalanceController::set("LFY", "position", Lypos);
    }
}

/**
 * Set a balance base line to get rid of the noise. This assumes
 * you start balancing when the robot is balanced. 
 */
void BalanceController::setBaseline(){
    getCurrentSupportPhase();
    ZMPcalculation();
    DSPControl();

    BaseDSP[0][0] = -1 * floor(ControlDSP[0][0]*1000) / 1000; 
    BaseDSP[0][1] = -1 * floor(ControlDSP[0][1]*1000) / 1000;
    BaseDSP[1][0] = -1 * floor(ControlDSP[1][0]*1000) / 1000;
    BaseDSP[1][1] = -1 * floor(ControlDSP[1][1]*1000) / 1000;
}

/**
 * initialize the Balance controller. This needs a copy of the 
 * state instance. 
 * @param theState The state of the robot. 
 */
void BalanceController::initBalanceController(HuboState& theState){

    state = &theState; 
    if(state == NULL)
    {
        cout << "Error initializing the Balance Controller, the state was never initalized." << endl;
        return; 
    }
    initialized = allComponentsFound(); // Check to see if the hubo state has all of the needed components

    if(!initialized){
        cout << "Error initializing the Balance Controller, Not all of the components needed to balance were found." << endl;
        return;
    }

    getCurrentSupportPhase(); //Find the initial support phase
    ZMPInitialization(); //Initialize the zmp
}

/**
 * Look to see if all the needed balancing components are in the robot.
 * @return True if all names existed. 
 */
bool BalanceController::allComponentsFound(){
    for(int i = 0; i < 9; i++)
    {
        if(!state->nameExists(balanceComponents[i])) //If the name does not exist
        {
            return false;
        }
    }
    return true; // All names existed
}

/**
 * Return the Zero Moment point in either the X or Y direction. 
 * @param  value 0 is the X direction and 1 is the Y direction
 * @return       The zmp value
 */
double BalanceController::getZMP(int value){
    return filteredZMP[value];
}

/**
 * Calculate the ZMP value
 */
void BalanceController::ZMPcalculation(){
    // Rx: Right x force component
    // Ry: Right y force component
    // Rz: Right z torque component
    // Lx: Left x force component
    // Ly: Left y force component
    // Lz: Left z torque component

    // RAx: Right ankle x position 
    // RAy: Right ankle y position
    // RAz: Right ankle z position
    // LAx: Left ankle x position
    // LAy: Left ankle y position
    // LAz: Left ankle z position
    double filteredZMPOld[6];
    double pelvis_width = 0.177;
    double alpha = 0.1570796; //alpha  2.0*PI*5.0f*5/1000.0
    
    double Rx = BalanceController::get("RAT", "m_x");
    double Ry = BalanceController::get("RAT", "m_y");
    double Rz = BalanceController::get("RAT", "f_z");
    double Lx = BalanceController::get("LAT", "m_x");
    double Ly = BalanceController::get("LAT", "m_y");
    double Lz = BalanceController::get("LAT", "f_z");
    double RAx = -1 * BalanceController::get("RFX", "position"); 
    double RAy = -1 * BalanceController::get("RFY", "position");
    double RAz = BalanceController::get("RFZ", "position");
    double LAx = -1 * BalanceController::get("LFX", "position");
    double LAy = -1 * BalanceController::get("LFY", "position");
    double LAz = BalanceController::get("LFZ", "position");

    double totalMX;    // total moment in the x
    double totalMY;    // total moment in the y

    // All calculations are from RAINBOW code
    if(Rz > 30 && Lz > 30)
    {
        //Double support
        totalMX = Rx + Lx + ((LAy+.5*pelvis_width)*Lz) + ((RAy-.5*pelvis_width)*Rz);  
        totalMY = Ry + Ly - (LAx*Lz) - (RAx*Rz);

        zmp[0] = -1000.0*totalMY/(Lz+Rz);
        zmp[1] = 1000.0*totalMX/(Lz+Rz);
        zmp[2] = -1000.0*Ry/Rz;
        zmp[3] = 1000.0*Rx/Rz;
        zmp[4] = -1000.0*Ly/Lz;
        zmp[5] = 1000.0*Lx/Lz;
    }
    else if(Rz > 30)
    {
        //Right leg support

        zmp[2] = -1000.0*Ry/Rz;
        zmp[3] = 1000.0*Rx/Rz;
        zmp[0] = RAx*1000.0 + zmp[2];
        zmp[1] = (RAy-0.5*pelvis_width)*1000.0 + zmp[3];
        zmp[4] = 0.0;
        zmp[5] = 0.0;
    }
    else if(Lz > 30)
    {
        //Left leg support
        zmp[4] = -1000.0*Ly/Lz;
        zmp[5] = 1000.0*Lx/Lz;
        zmp[0]  = LAx*1000.0 + zmp[4];
        zmp[1]  = (LAy+0.5*pelvis_width)*1000.0 + zmp[5];   
        zmp[2] = 0.0;
        zmp[3] = 0.0;
    }


    for(int i=0 ; i<6 ; i++)
    {
        filteredZMPOld[i] = filteredZMP[i];
        filteredZMP[i] = (1.0f-alpha) * filteredZMPOld[i] + alpha * zmp[i];
    }
}

/**
 * calculate the current support phase
 */
void BalanceController::getCurrentSupportPhase(){
    double Rz = get("RAT", "f_z");
    double Lz = get("LAT", "f_z");

    if(Rz > 30 && Lz > 30)
    {
        phase = BOTH_FEET;
    }
    else if(Rz > 30)
    {
        phase = RIGHT_FOOT;
    }
    else if(Lz > 30)
    {
        phase = LEFT_FOOT;
    }
}

/**
 * Set the offset of a joint to a value
 * @param name   The joint to set the offset on
 * @param offset The offset to set
 */
void BalanceController::setOffset(string name, double offset){
    if (!state->nameExists(name)){
        cout << "Error. No component with name " << name << " registered. Aborting." << endl;
        return;
    }



    if (!static_cast<MetaJoint*>(state->getComponent(name))->setOffset(offset)){
        cout << "Error setting offset of component " << name << endl;
        return;
    }
}

/* Code duplication. I know. I should really fix it */
double BalanceController::get(string name, string property){

    if (!state->nameExists(name)){
        cout << "Error. No component with name " << name << " registered. Aborting." << endl;
        return 0;
    }

    Properties properties = Names::getProps();

    if (properties.count(property) == 0){
        cout << "Error. No property with name " << property << " registered. Aborting." << endl;
        return 0;
    }

    double result = 0;

    if (!state->getComponent(name)->get(properties[property], result)){
        cout << "Error getting property " << property << " of component " << name << endl;
        return 0;
    }

    return result;
}
/* More duplication. I know it's bad :( */
void BalanceController::set(string name, string property, double value){
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

/* just a little more code duplication. I'll try to fix it when I get something to work */
bool BalanceController::requiresMotion(string name){
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

    return fabs(step - goal) > .001;
}

/**
 * Does the Digital Signal Processing of all the data it has. 
 * This is what generates the offsets for the joints and does all of the math. 
 * It was taken almost directly from RAINBOW. I removed some stuff we don't use.
 */
void BalanceController::DSPControl(){

    unsigned char i;
    unsigned char Command = 1;
    static float x1new[2] = {0.0f, 0.0f}, x2new[2] = {0.0f, 0.0f};
    static float x1[2] = {0.0f, 0.0f}, x2[2] = {0.0f, 0.0f};
    float delZMP[2] = {zmp[0]+InitZmp[0], zmp[1]+InitZmp[1]};
    static float gainOveriding = 0.0f;
    float controlOutput[2];
    float dspLimit = 100.0f;
    float gain = 0.8;

    const float adm[2][4] = {0.519417298104f,   -2.113174135817f,   0.003674390567f,    0.994144954175f,    0.734481292627f,    -1.123597885586f,   0.004304846434f,    0.997046817458f};
    const float bdm[2][2] = {0.003674390567f,   0.000010180763f,    0.004304846434f,    0.000011314544f};
    const float cdm[2][2] = {9.698104154920f,   -126.605319064208f, -0.555683927701f,   -77.652283056185f};
    
    for(i=0 ; i<2 ; i++)
    {
        x1new[i] = adm[i][0]*x1[i] + adm[i][1]*x2[i] + bdm[i][0]*delZMP[i];
        x2new[i] = adm[i][2]*x1[i] + adm[i][3]*x2[i] + bdm[i][1]*delZMP[i];
        controlOutput[i] = cdm[i][0]*x1new[i] + cdm[i][1]*x2new[i];
        
        x1[i] = x1new[i];   x2[i] = x2new[i];
        
        if(controlOutput[i] > dspLimit) controlOutput[i] = dspLimit;
        else if(controlOutput[i] < -dspLimit) controlOutput[i] = -dspLimit;
        else;
        
        ControlDSP[0][i] = gainOveriding*gain*controlOutput[i]/1000.0f;
        ControlDSP[1][i] = gainOveriding*gain*controlOutput[i]/1000.0f;
    }
    gainOveriding += 0.05;
    if(gainOveriding > 1.0) gainOveriding = 1.0;
}

/**
 * Damping control from rainbow. This isn't used. 
 * @return nothing. Oops! 
 */
double BalanceController::DampingControl(){
    float tempControlAngle[2];
    float limitAngle = 20.0f;
    float gain[6];
    float controlDampCutoff = 1.0f;

    double roll         = get("IMU", "x_rot");  
    double pitch        = get("IMU", "y_rot");
    double roll_vel     = get("IMU", "x_acc"); 
    double pitch_vel    = get("IMU", "y_acc");
    double oldDampingAngle[4];

    double alpha = 0.0314159;        //2.0f*PI*controlDampCutoff*INT_TIME/1000.0f

    unsigned char i;
    for(i=0 ; i<6 ; i++) gain[i] = dampingGain[i];

    for(int i=0; i < 4; i ++)
    {
        oldDampingAngle[i] = Damping[i];
    }
    
    tempControlAngle[0] = gain[0]*(-gain[1]*pitch + gain[2]*pitch_vel);
    tempControlAngle[1] = gain[3]*(-gain[4]*roll + gain[5]*roll_vel);

    switch(phase)
    {
    case RIGHT_FOOT:
        Damping[0] = (float)((1.0f-alpha)*oldDampingAngle[0]+alpha*tempControlAngle[0]);
        oldDampingAngle[0] = Damping[0];
        Damping[1] = (float)((1.0f-alpha)*oldDampingAngle[1]+alpha*tempControlAngle[1]);
        oldDampingAngle[1] = Damping[1];
        break;
    case LEFT_FOOT:
        Damping[2] = (float)((1.0f-alpha)*oldDampingAngle[2]+alpha*tempControlAngle[0]);
        oldDampingAngle[2] = Damping[2];
        Damping[3] = (float)((1.0f-alpha)*oldDampingAngle[3]+alpha*tempControlAngle[1]);
        oldDampingAngle[3] = Damping[3];
        break;
    case BOTH_FEET:
    default:
        tempControlAngle[0] = 0.0f;
        tempControlAngle[1] = 0.0f;
        Damping[0] = (float)((1.0f-alpha)*oldDampingAngle[0]+alpha*tempControlAngle[0]);
        oldDampingAngle[0] = Damping[0];
        Damping[1] = (float)((1.0f-alpha)*oldDampingAngle[1]+alpha*tempControlAngle[1]);
        oldDampingAngle[1] = Damping[1];
        Damping[2] = (float)((1.0f-alpha)*oldDampingAngle[2]+alpha*tempControlAngle[0]);
        oldDampingAngle[2] = Damping[2];
        Damping[3] = (float)((1.0f-alpha)*oldDampingAngle[3]+alpha*tempControlAngle[1]);
        oldDampingAngle[3] = Damping[3];
        break;
    }

    if(Damping[0] > limitAngle) Damping[0] = limitAngle;
    else if(Damping[0] < -limitAngle) Damping[0] = -limitAngle;
    if(Damping[2] > limitAngle) Damping[2] = limitAngle;
    else if(Damping[2] < -limitAngle) Damping[2] = -limitAngle;
    
    if(Damping[1] > limitAngle) Damping[1] = limitAngle;
    else if(Damping[1] < -limitAngle) Damping[1] = -limitAngle;
    if(Damping[3] > limitAngle) Damping[3] = limitAngle;
    else if(Damping[3] < -limitAngle) Damping[3] = -limitAngle;
}

// doesn't really do anything :( 
void BalanceController::ZMPInitialization(){
    unsigned char i;
    float KI;
    float zmpTemp[2], heightTemp, pitchHipTemp;
    double roll = get("IMU", "x_rot");
    double pitch = get("IMU", "y_rot");

    // _zmpInit[]
    // 0: torso x-direction
    // 1: torso y-direction
    // 2: z-direction of the foot
    // 3: hip pitch angle
    // 4: ankle pitch angle
    // 5: right ankle roll angle
    // 6: left ankle roll angle

    // 0:Right 1:Left
    // 0:X 1:Y 2:Z
    // [Right/Left][X/Y/Z]
    

    // ZMP initialization using the torso center position // Adjusts the X and Y offsets 
    KI = 0.005/5;
    
    for(i=0 ; i<2 ; i++) // For X and Y 
    {
        zmpTemp[i] = (InitZmp[i] - zmp[i])/1000.0; // Create the average zmp 
        
        if(zmpTemp[i] > 0.05) zmpTemp[i] = 0.05;
        else if(zmpTemp[i] < -0.05) zmpTemp[i] = -0.05;
        InitRefAngles[0][i] -= KI*zmpTemp[i];
        InitRefAngles[1][i] -= KI*zmpTemp[i];
    }

    // body roll angle initialization using the foot position(z-direction) // Changes the height
    KI = 0.004/5;

    heightTemp = roll/1000.0;
    
    if(heightTemp > 0.001) heightTemp = 0.001;
    else if(heightTemp < -0.001) heightTemp = -0.001;

    InitRefAngles[0][2] -= KI*heightTemp;
    InitRefAngles[1][2] += KI*heightTemp;
    
    // body pitch angle initialization using the hip pitch angles
    KI = 0.01;
    pitchHipTemp = pitch;///1000.0f;
        
    if(pitchHipTemp > 0.01) pitchHipTemp = 0.01;
    else if(pitchHipTemp < -0.01) pitchHipTemp = -0.01;
    
    //_joint[RAP].RefAngleFF += KI*pitchHipTemp;
    //_joint[LAP].RefAngleFF += KI*pitchHipTemp;
    double RHP_ref = get("RHP", "position");
    double LHP_ref = get("LHP", "position");
    double RAP_ref = get("RAP", "position");
    double LAP_ref = get("LAP", "position");
    double RAR_ref = get("RAR", "position");
    double LAR_ref = get("LAR", "position");
    double Lmy = get("LAT", "m_y");
    double Rmy = get("RAT", "m_y");
    double Lmx = get("LAT", "m_x");
    double Rmx = get("RAT", "m_x");

    RHP_ref += KI*pitchHipTemp;
    LHP_ref += KI*pitchHipTemp;
    
    // moment at ankle(My) initialization using ankle pitch angles
    //KI=0.0001f;
    KI = 0.0002;
    RAP_ref += KI*(Rmy-Lmy);

    // moment at ankle(Mx) initialization using ankle roll angles
    KI = 0.02/5; 
    RAR_ref += KI*Rmx;
    LAR_ref += KI*Lmx;

     // Initial joint off sets 
    // Nothing happens because I don't quite trust it yet... 
}
