#include "HAP_ARM.h"

HAP_ARM::HAP_ARM(QWidget *parent):
QWidget(parent)
{
    cout << "Building haptic environment." << endl;

	///////////////////////////////////////////////////////////////////////////////
    world = new cWorld();
    world->setBackgroundColor((200/255.0),(200/255.0),(200/255.0));
    world->setShowFrame(1);
    world->setFrameSize(100.0,100.0,true);

    camera = new cCamera(world);
    world->addChild(camera);
    camera_pos_start.set(0, -100, 800);
    camera_target_start.set(0.0, -100.0, 0.0);
    camera_up_start.set(0,1,0);
    camera->set(camera_pos_start, camera_target_start, camera_up_start);
    camera->setClippingPlanes(0.1, 7000.0);
    camera->enableMultipassTransparency(true);

    light = new cLight(world);
    camera->addChild(light);                   // attach light to camera
    light->setEnabled(true);                   // enable light source
    light->setPos(camera_pos_start);  // position the light source
    light->setDir(camera_target_start);  // define the direction of the light beam
	///////////////////////////////////////////////////////////////////////////////

	/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	///////////////////////////////////////////////////////////////////////////////GUI stuff has to come after the creation of the camera
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,835,355);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

	box_title = new QLabel("HAP_ARM",this);
	box_title->setGeometry(5,5,150,30);
	box_title->setFont(QFont("Times", 12, QFont::Bold));

	forces_text_label_assistoBot = new QLabel("", this);
    forces_text_label_assistoBot->setGeometry(QRect(5, 40, 300, 60));

	pos_text_label_assistoBot = new QLabel("", this);
    pos_text_label_assistoBot->setGeometry(QRect(5, 105, 300, 60));

	forces_text_label_needleBot = new QLabel("", this);
    forces_text_label_needleBot->setGeometry(QRect(5, 170, 300, 60));

	pos_text_label_needleBot = new QLabel("", this);
    pos_text_label_needleBot->setGeometry(QRect(5, 235, 300, 120));

	left_handedness_button = new QRadioButton("Right Handed", this);
    left_handedness_button->setGeometry(QRect(325, 5, 200, 20));
    left_handedness_button->setAutoExclusive(0);
    left_handedness_flag = 0; //Start out right-handed by default.
	left_handedness_button->setChecked(left_handedness_flag);
	connect(left_handedness_button, SIGNAL(toggled(bool)), this, SLOT(change_handedness(bool)));

	int enable_buttons_start_x = 325;
	int enable_buttons_start_y = 30;
	int enable_buttons_width = 200;
	int enable_buttons_width_inc = 10;
	int enable_buttons_height = 20;
	int enable_buttons_height_inc = 20;

	//////////////////////////////////////////assistoBot
	enable_all_forces_assistoBot_flag = 0;
    enable_all_forces_assistoBot_button = new QPushButton("Press to Disable AssistoBot Forces", this);
    enable_all_forces_assistoBot_button->setGeometry(QRect(enable_buttons_start_x, enable_buttons_start_y + 0*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));
    connect(enable_all_forces_assistoBot_button, SIGNAL(clicked()), this, SLOT(change_enabling_all_forces_assistoBot()));

    enable_spring_force_assistoBot_button = new QRadioButton("Enable AssistoBot Spring Force", this);
    enable_spring_force_assistoBot_button->setGeometry(QRect(enable_buttons_start_x, enable_buttons_start_y + 1*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));
    enable_spring_force_assistoBot_button->setAutoExclusive(0);
    enable_spring_force_assistoBot_flag = 0;
	enable_spring_force_assistoBot_button->setChecked(enable_spring_force_assistoBot_flag);
	
	enable_chai_model_interaction_force_assistoBot_button = new QRadioButton("Enable AssistoBot Chai Int Force", this);
    enable_chai_model_interaction_force_assistoBot_button->setGeometry(QRect(enable_buttons_start_x, enable_buttons_start_y + 2*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));
    enable_chai_model_interaction_force_assistoBot_button->setAutoExclusive(0);
    enable_chai_model_interaction_force_assistoBot_flag = 0;
	enable_chai_model_interaction_force_assistoBot_button->setChecked(enable_chai_model_interaction_force_assistoBot_flag);
	
    enable_ATI_force_assistoBot_button = new QRadioButton("Enable AssistoBot ATI Force", this);
    enable_ATI_force_assistoBot_button->setGeometry(QRect(enable_buttons_start_x, enable_buttons_start_y + 3*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));
    enable_ATI_force_assistoBot_button->setAutoExclusive(0);
    enable_ATI_force_assistoBot_flag = 0;
	enable_ATI_force_assistoBot_button->setChecked(enable_ATI_force_assistoBot_flag);
	
	spring_force_gain_assistoBot = 0.5;
	spring_force_gain_assistoBot_spinbox = new print_labeled_spinbox(this, "A Spring Force Gain: ", 0.001, 3, 0.0, 10.0, spring_force_gain_assistoBot);
	spring_force_gain_assistoBot_spinbox->setGeometry(QRect(enable_buttons_start_x, enable_buttons_start_y + 4*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));

	ATI_gain_assistoBot = 1.0;
	ATI_gain_assistoBot_spinbox = new print_labeled_spinbox(this, "A ATI    Force Gain: ", 0.001, 3, 0.0, 10.0, ATI_gain_assistoBot);
	ATI_gain_assistoBot_spinbox->setGeometry(QRect(enable_buttons_start_x, enable_buttons_start_y + 5*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));

	chai_model_interaction_gain_assistoBot = 1.0;
	chai_model_interaction_gain_assistoBot_spinbox = new print_labeled_spinbox(this, "A CHAI   Force Gain: ", 0.001, 3, 0.0, 10.0, chai_model_interaction_gain_assistoBot);
	chai_model_interaction_gain_assistoBot_spinbox->setGeometry(QRect(enable_buttons_start_x, enable_buttons_start_y + 6*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));
	//////////////////////////////////////////assistoBot

	//////////////////////////////////////////needleBot
	enable_all_forces_needleBot_flag = 0;
    enable_all_forces_needleBot_button = new QPushButton("Press to Disable NeedleBot Forces", this);
    enable_all_forces_needleBot_button->setGeometry(QRect(enable_buttons_start_x + enable_buttons_width + enable_buttons_width_inc, enable_buttons_start_y + 0*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));
    connect(enable_all_forces_needleBot_button, SIGNAL(clicked()), this, SLOT(change_enabling_all_forces_needleBot()));

    enable_spring_force_needleBot_button = new QRadioButton("Enable NeedleBot Spring Force", this);
    enable_spring_force_needleBot_button->setGeometry(QRect(enable_buttons_start_x + enable_buttons_width + enable_buttons_width_inc, enable_buttons_start_y + 1*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));
    enable_spring_force_needleBot_button->setAutoExclusive(0);
    enable_spring_force_needleBot_flag = 0;
	enable_spring_force_needleBot_button->setChecked(enable_spring_force_needleBot_flag);
	
	enable_chai_model_interaction_force_needleBot_button = new QRadioButton("Enable NeedleBot Chai Int Force", this);
    enable_chai_model_interaction_force_needleBot_button->setGeometry(QRect(enable_buttons_start_x + enable_buttons_width + enable_buttons_width_inc, enable_buttons_start_y + 2*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));
    enable_chai_model_interaction_force_needleBot_button->setAutoExclusive(0);
    enable_chai_model_interaction_force_needleBot_flag = 0;
	enable_chai_model_interaction_force_needleBot_button->setChecked(enable_chai_model_interaction_force_needleBot_flag);
	
    enable_ATI_force_needleBot_button = new QRadioButton("Enable NeedleBot ATI Force", this);
    enable_ATI_force_needleBot_button->setGeometry(QRect(enable_buttons_start_x + enable_buttons_width + enable_buttons_width_inc, enable_buttons_start_y + 3*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));
    enable_ATI_force_needleBot_button->setAutoExclusive(0);
    enable_ATI_force_needleBot_flag = 0;
	enable_ATI_force_needleBot_button->setChecked(enable_ATI_force_needleBot_flag);
	
	spring_force_gain_needleBot = 1.0;
	spring_force_gain_needleBot_spinbox = new print_labeled_spinbox(this, "N Spring Force Gain: ", 0.001, 3, 0.0, 10.0, spring_force_gain_needleBot);
	spring_force_gain_needleBot_spinbox->setGeometry(QRect(enable_buttons_start_x + enable_buttons_width + enable_buttons_width_inc, enable_buttons_start_y + 4*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));

	ATI_gain_needleBot = 1.0;
	ATI_gain_needleBot_spinbox = new print_labeled_spinbox(this, "N ATI    Force Gain: ", 0.001, 3, 0.0, 10.0, ATI_gain_needleBot);
	ATI_gain_needleBot_spinbox->setGeometry(QRect(enable_buttons_start_x + enable_buttons_width + enable_buttons_width_inc, enable_buttons_start_y + 5*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));

	chai_model_interaction_gain_needleBot = 1.0;
	chai_model_interaction_gain_needleBot_spinbox = new print_labeled_spinbox(this, "N CHAI   Force Gain: ", 0.001, 3, 0.0, 10.0, chai_model_interaction_gain_needleBot);
	chai_model_interaction_gain_needleBot_spinbox->setGeometry(QRect(enable_buttons_start_x + enable_buttons_width + enable_buttons_width_inc, enable_buttons_start_y + 6*(enable_buttons_height + enable_buttons_height_inc), enable_buttons_width, enable_buttons_height));
	//////////////////////////////////////////needleBot

    animation_width = 500;
    animation_height = 500;
    animation_x = 2560-800;
    animation_y = -1440 + 5;//-1440*2 + 10;
    animation = new my_QGLWidget(camera, 0, 0, string("Click and Drag Mouse to Rotate"));
    animation->setGeometry(QRect(animation_x, animation_y, animation_width, animation_height));

	Qt::WindowFlags flags = animation->windowFlags();
	animation->setWindowFlags(flags | Qt::WindowStaysOnTopHint);

    animation->show();
	///////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////
	handler = new cHapticDeviceHandler();
	cout << "num devices: " << handler->getNumDevices() << endl;
    hapticDevice_assistoBot = new cGenericHapticDevice();
    handler->getDevice(hapticDevice_assistoBot, 0);
		info_assistoBot = hapticDevice_assistoBot->getSpecifications();
		cout << "Acquired Haptic Device " << info_assistoBot.m_modelName << endl;
    tool_assistoBot = new cGeneric3dofPointer(world);
    world->addChild(tool_assistoBot);
    tool_assistoBot->setHapticDevice(hapticDevice_assistoBot);
    //tool_assistoBot->m_deviceGlobalPos.set(0,0,100); //so that we don't start in the middle of the model
    tool_assistoBot->start();
    tool_assistoBot->initialize(); //so that we don't start in the middle of the model.  must come after setting m_deviceGlobalPos and starting tool.
    tool_assistoBot->setShowEnabled(1);
    double proxyRadius = 0.01;
    tool_assistoBot->setRadius(3);
    tool_assistoBot->setForcesON();
	tool_assistoBot->setShowFrame(true, false);
	tool_assistoBot->setFrameSize(100.0,100.0,true);
 
    hapticDevice_needleBot = new cGenericHapticDevice();
    handler->getDevice(hapticDevice_needleBot, 1);
		info_needleBot = hapticDevice_needleBot->getSpecifications();
		cout << "Acquired Haptic Device " << info_needleBot.m_modelName << endl;
    tool_needleBot = new cGeneric3dofPointer(world);
    world->addChild(tool_needleBot);
    tool_needleBot->setHapticDevice(hapticDevice_needleBot);
    //tool_needleBot->m_deviceGlobalPos.set(0,0,100); //so that we don't start in the middle of the model
    tool_needleBot->start();
    tool_needleBot->initialize(); //so that we don't start in the middle of the model.  must come after setting m_deviceGlobalPos and starting tool.
    tool_needleBot->setShowEnabled(1);
    tool_needleBot->setRadius(3);
    tool_needleBot->setForcesON();

    double workspaceScaleFactor = tool_assistoBot->getWorkspaceScaleFactor();
    double stiffnessMax = info_assistoBot.m_maxForceStiffness / workspaceScaleFactor;
    double forceMax = info_assistoBot.m_maxForce;
    double dampingMax = info_assistoBot.m_maxLinearDamping / workspaceScaleFactor;

  //  double stiffnessMax = 1.0; ///////delete///////
    double virtualWallStiffnessGain = 0.001; ///////delete///////
    model_center.set(0.0, 0.0, 0.0);


    add_chai_model(cmesh_model_vec, cmesh_model_names, world, "link_1.obj", "link_1", 1000.0, proxyRadius, virtualWallStiffnessGain*stiffnessMax, model_center, true, cVector3d(0.5, 0.5, 0.5), 1.0);
    add_chai_model(cmesh_model_vec, cmesh_model_names, world, "link_2.obj", "link_2", 1000.0, proxyRadius, virtualWallStiffnessGain*stiffnessMax, model_center, true, cVector3d(0.5, 0.5, 0.5), 1.0);
    add_chai_model(cmesh_model_vec, cmesh_model_names, world, "link_3.obj", "link_3", 1000.0, proxyRadius, virtualWallStiffnessGain*stiffnessMax, model_center, true, cVector3d(0.5, 0.5, 0.5), 1.0);
    add_chai_model(cmesh_model_vec, cmesh_model_names, world, "link_4.obj", "link_4", 1000.0, proxyRadius, virtualWallStiffnessGain*stiffnessMax, model_center, true, cVector3d(0.5, 0.5, 0.5), 1.0);
    add_chai_model(cmesh_model_vec, cmesh_model_names, world, "assistoBot_last_link_final.obj", "link_5", 1000.0, proxyRadius, virtualWallStiffnessGain*stiffnessMax, model_center, true, cVector3d(0.5, 0.5, 0.5), 1.0);
    /////add_chai_model(cmesh_model_vec, cmesh_model_names, world, "assistoBot_chai_workspace.obj", "workspace", 1000.0, proxyRadius, virtualWallStiffnessGain*stiffnessMax, model_center, false, cVector3d(0.5, 0.5, 0.5), 0.1);
    //add_chai_model(cmesh_model_vec, mainWindow->cmesh_model_names, mainWindow->world_1, "workspace.obj", "workspace", 1000.0, proxyRadius, virtualWallStiffnessGain*stiffnessMax, cVector3d(1, 157, -200), false, cVector3d(200.0/255.0, 200.0/255.0, 200.0/255.0), 0.0);

    force_chaiFrame_line_1 = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    world->addChild(force_chaiFrame_line_1);
    force_chaiFrame_line_1->m_ColorPointA.set(0,0,1,1);
    force_chaiFrame_line_1->m_ColorPointB.set(0,0,1,1);
    force_chaiFrame_line_1->setShowEnabled(true);


    tool_assistoBot->setWorkspaceRadius(100);
	tool_needleBot->setWorkspaceRadius(100);

    cMatrix3d frame_shift;
    frame_shift.set(0, 1, 0, -1, 0, 0, 0, 0, 1);
    //tool_assistoBot->setRot(frame_shift); ///////////////////////////////// WE'RE APPLYING THIS FRAM SHIFT TO MAKE IT WORK WITH THE ASSISTOBOT. CONFIRM THAT THIS IS NEEDED.
    tool_assistoBot->setPos(0, 0, 0); //This offset allows for us to set where in the omni's workspace the maxium reach of assistoBot is.

    tool_needleBot->setPos(0, 0, 0); //This offset allows for us to set where in the omni's workspace the maxium reach of assistoBot is.


    L1 = 60;
    L2 = 80;
    L3 = 40;
    L4 = 241.27;

    assistoBot_central_axis.set(0.0, -350.0, 0.0);


	
	updateAndApplyForce(cVector3d(0,0,0), cVector3d(0,0,0), std::vector<double>(3,0), cVector3d(0,0,0), cVector3d(0,0,0), std::vector<double>(3,0)); //Must first write a zero force to the device to be able to render forces.


	PosNeedleBot = std::vector<double>(7, 0);
	VelNeedleBot = std::vector<double>(7, 0);
	PosAssistoBot = std::vector<double>(3, 0);



    cout << "Done building haptic environment." << endl;
}


void HAP_ARM::updateAndApplyForce(cVector3d commandedPos_assistoBot, cVector3d actualPos_assistoBot, std::vector<double> ATI_assistoBot_force_vec_in, cVector3d commandedPos_needleBot, cVector3d actualPos_needleBot, std::vector<double> ATI_needleBot_force_vec_in)
{
	//////////////////////////////////////////////////assistoBot
    tool_assistoBot->computeInteractionForces();

	spring_force_assistoBot = spring_force_gain_assistoBot*(actualPos_assistoBot - commandedPos_assistoBot);
	//cout << "AssistoBot Spring Force: X: " << spring_force_assistoBot[0] << " Y: " << spring_force_assistoBot[1] << " Z: " << spring_force_assistoBot[2] << endl;
    tool_assistoBot->m_lastComputedGlobalForce += enable_spring_force_assistoBot_flag*spring_force_assistoBot;

	tool_assistoBot->m_lastComputedGlobalForce[0] += enable_ATI_force_assistoBot_flag*ATI_gain_assistoBot*ATI_assistoBot_force_vec_in[0];
	tool_assistoBot->m_lastComputedGlobalForce[1] += enable_ATI_force_assistoBot_flag*ATI_gain_assistoBot*ATI_assistoBot_force_vec_in[1];
	tool_assistoBot->m_lastComputedGlobalForce[2] += enable_ATI_force_assistoBot_flag*ATI_gain_assistoBot*ATI_assistoBot_force_vec_in[2];

	tool_assistoBot->applyForces();
	//////////////////////////////////////////////////assistoBot

	//////////////////////////////////////////////////needleBot
	tool_needleBot->computeInteractionForces();

	spring_force_needleBot= spring_force_gain_needleBot*(actualPos_needleBot - commandedPos_needleBot);
    tool_needleBot->m_lastComputedGlobalForce += enable_spring_force_needleBot_flag*spring_force_needleBot;

	tool_needleBot->m_lastComputedGlobalForce[0] += enable_ATI_force_needleBot_flag*ATI_gain_needleBot*ATI_needleBot_force_vec_in[0];
	tool_needleBot->m_lastComputedGlobalForce[1] += enable_ATI_force_needleBot_flag*ATI_gain_needleBot*ATI_needleBot_force_vec_in[1];
	tool_needleBot->m_lastComputedGlobalForce[2] += enable_ATI_force_needleBot_flag*ATI_gain_needleBot*ATI_needleBot_force_vec_in[2];

	tool_needleBot->applyForces();
	//////////////////////////////////////////////////needleBot
}



void HAP_ARM::updateHapticPose(void)
{
    world->computeGlobalPositions(true);
    tool_assistoBot->updatePose();
	tool_needleBot->updatePose();

    toolPos_assistoBot = tool_assistoBot->m_deviceGlobalPos;
    toolVel_assistoBot = tool_assistoBot->m_deviceGlobalVel;
	toolRot_assistoBot = tool_assistoBot->m_deviceGlobalRot; /////////////////replace this!!!! 

    toolPos_needleBot = tool_needleBot->m_deviceGlobalPos;
    toolVel_needleBot = tool_needleBot->m_deviceGlobalVel;
	toolRot_needleBot = tool_needleBot->m_deviceGlobalRot; /////////////////replace this!!!! 

	///////////////////////////////////////////////////////////////////////////////////////////
	//Compute yaw, pitch, roll of omni in robot frame 
    /////////////////////////////////////////////////////////////////////////////////////////// 

    for(int i = 0; i < 3; i++) 
    { 
        for(int j = 0; j < 3; j++) 
        { 
            rot_matrix_CML_needleBot(i,j) = toolRot_needleBot[i][j];  
        } 
    } 
 
    //Convert rotational matrix to yaw, pitch, roll 
	float tempRoll, tempPitch, tempYaw;
    cml::matrix_to_euler(rot_matrix_CML_needleBot, tempRoll, tempPitch, tempYaw, cml::euler_order_xyz, 0.01f);  //Rotations in order of left to right about global coordinates.

	double compBeta, compAlpha, compGamma;
	compBeta = atan2(-1.0*toolRot_needleBot[3][1], sqrt(pow(toolRot_needleBot[1][1], 2) + pow(toolRot_needleBot[2][1], 2)));
	compAlpha = atan2(toolRot_needleBot[2][1]/cos(compBeta), toolRot_needleBot[1][1]/cos(compBeta));
	compGamma = atan2(toolRot_needleBot[3][2]/cos(compBeta), toolRot_needleBot[3][3]/cos(compBeta));

    //Set yaw, pitch, roll 
	tempRoll = tempRoll*180.0/pi_define; 
	tempPitch = (tempPitch*180.0/pi_define + 90.0);
	tempYaw = 1.0*(tempYaw*180.0/pi_define + 0.0); 
    /////////////////////////////////////////////////////////////////////////////////////////// 
     
    //Get omni buttons 
    /////////////////////////////////////////////////////////////////////////////////////////// 
    button_0_state_assistoBot = tool_assistoBot->getUserSwitch(0); 
    button_1_state_assistoBot = tool_assistoBot->getUserSwitch(1); 

    button_0_state_needleBot = tool_needleBot->getUserSwitch(0); 
    button_1_state_needleBot = tool_needleBot->getUserSwitch(1); 
   
	double cathIns_button_integrator_rate = 0.001;
    if(button_0_state_needleBot != 0) 
    { 
		if(PosNeedleBot[6] <= rd_needle_driver_stringpot_PositionMax - cathIns_button_integrator_rate)
		{
			PosNeedleBot[6] = PosNeedleBot[6] + cathIns_button_integrator_rate; 
		}
		else
		{
			PosNeedleBot[6] = rd_needle_driver_stringpot_PositionMax;
		}
    } 
    if(button_1_state_needleBot != 0) 
    { 
		if(PosNeedleBot[6] >= cathIns_button_integrator_rate)
		{
			PosNeedleBot[6] = PosNeedleBot[6] - cathIns_button_integrator_rate; 
		}
		else
		{
			PosNeedleBot[6] = 0;
		}
    } 

	PosNeedleBot[0] = -1*tool_needleBot->m_deviceGlobalPos.y;
	PosNeedleBot[1] = 1*tool_needleBot->m_deviceGlobalPos.x;
	PosNeedleBot[2] = tool_needleBot->m_deviceGlobalPos.z;
	PosNeedleBot[3] = tempYaw;
	PosNeedleBot[4] = tempPitch;
	PosNeedleBot[5] = tempRoll;

	PosAssistoBot[0] = 1*tool_assistoBot->m_deviceGlobalPos.x;
	PosAssistoBot[1] = 1*tool_assistoBot->m_deviceGlobalPos.y;
	PosAssistoBot[2] = tool_assistoBot->m_deviceGlobalPos.z;

}

void HAP_ARM::DisplayKinematics(cVector3d jointTrad, cVector3d EndEff)
{
        cMatrix3d R1 = cRotMatrix(cVector3d(0,0,1), jointTrad[1]); //T1
        cmesh_model_vec[0]->setRot(R1);
        cmesh_model_vec[0]->setPos(cVector3d(assistoBot_central_axis[0],assistoBot_central_axis[1],EndEff[2]));

        cMatrix3d R2 = cRotMatrix(cVector3d(0,0,1), jointTrad[2]); //T2
        cmesh_model_vec[1]->setRot(R2);
        cmesh_model_vec[1]->setPos(cVector3d(assistoBot_central_axis[0],assistoBot_central_axis[1],EndEff[2]));

        cVector3d link2_base_pos;
        link2_base_pos.x = L1*cos(jointTrad[1]) + assistoBot_central_axis[0];
        link2_base_pos.y = L1*sin(jointTrad[1]) + assistoBot_central_axis[1];
        link2_base_pos.z = EndEff[2];
        cmesh_model_vec[2]->setPos(link2_base_pos);
        cmesh_model_vec[2]->setRot(R2);

        cVector3d link3_base_pos;
        link3_base_pos.x = L2*cos(jointTrad[2]) + assistoBot_central_axis[0];
        link3_base_pos.y = L2*sin(jointTrad[2]) + assistoBot_central_axis[1];
        link3_base_pos.z = EndEff[2];
        cmesh_model_vec[3]->setPos(link3_base_pos);
        cmesh_model_vec[3]->setRot(R1);

        cVector3d link4_base_pos;
        link4_base_pos.x = EndEff[0];
        link4_base_pos.y = EndEff[1];
        link4_base_pos.z = EndEff[2];
        cmesh_model_vec[4]->setPos(link4_base_pos);
}

cVector3d HAP_ARM::InvKinematics(cVector3d EndEff)
{
    cVector3d jointTrad;

    cVector3d EndEffmod;
    EndEffmod[0] = EndEff[0] - assistoBot_central_axis[0];
    EndEffmod[1] = EndEff[1] - assistoBot_central_axis[1];

    double Px = EndEffmod[0];
    double Py = EndEffmod[1] - L4;

    double R = sqrt(Px*Px + Py*Py);
    double B = 1*acos( (pow(L1 + L3, 2) + L2*L2 - R*R) / (2*L2*(L1 + L3)) ) - pi_define;

    if(B*180/pi_define < -180)
    {
        B = -1*acos( (pow(L1 + L3, 2) + L2*L2 - R*R) / (2*L2*(L1 + L3)) ) - pi_define;
    }

    jointTrad[0] = EndEff[2]*(2.0*pi_define/10.0); //converts mm of linear slide to radian turns of lead screw
    jointTrad[2] = pi_define - asin( (  Py*(L2 + cos(B)*(L1 + L3)) - Px*sin(B)*(L1 + L3)  )  /  (L2*L2 + pow(L1 + L3, 2) + 2*L2*cos(B)*(L1 + L3)) );
    jointTrad[1] = B  + jointTrad[2];

    return jointTrad;
}

cVector3d HAP_ARM::FwdKinematics(cVector3d jointTrad)
{
    cVector3d EndEff;

    EndEff[0] = L2*cos(jointTrad[2]) + (L1 + L3)*cos(jointTrad[1]) + assistoBot_central_axis[0];
    EndEff[1] = L2*sin(jointTrad[2]) + (L1 + L3)*sin(jointTrad[1]) + L4 + assistoBot_central_axis[1];
    EndEff[2] = jointTrad[0]/(2.0*pi_define/10.0); //converts radian turns of lead screw to mm of linear slide;

    return EndEff;
}

cVector3d HAP_ARM::convertAngle2steps(cVector3d jointTrad)
{
    cVector3d jointTsteps;

    stepper_gear_ratio_rad_to_full_step_vec[0] = (1.0/0.9)*(180.0/pi_define); //400 full steps/rev (0.9deg/step) for lin engineering 5709x-01sd stepper motor
    stepper_gear_ratio_rad_to_full_step_vec[1] = (85.0/5.0)*(1.0/1.8)*(180.0/pi_define); //200 full steps/rev (1.8deg/step) for pololu 1209 stepper motor
    stepper_gear_ratio_rad_to_full_step_vec[2] = -1*(85.0/5.0)*(1.0/1.8)*(180.0/pi_define); //200 full steps/rev (1.8deg/step) for pololu 1209 stepper motor, MINUS sign reverses direction of motor correctly

    jointThomingOffsetRad[0] = 96.674*(2*pi_define/10.0);;
    jointThomingOffsetRad[1] = (45.16*pi_define/180.0);
    jointThomingOffsetRad[2] = (151.82*pi_define/180.0);

    for(int i = 0; i < 3; i++)
    {
        jointTsteps[i] = stepper_gear_ratio_rad_to_full_step_vec[i]*(jointTrad[i] - jointThomingOffsetRad[i]);
    }

    return jointTsteps;
}

cVector3d HAP_ARM::convertSteps2angle(cVector3d jointTsteps)
{
    cVector3d jointTrad;

    for(int i = 0; i < 3; i++)
    {
        jointTrad[i] = (jointTsteps[i]/stepper_gear_ratio_rad_to_full_step_vec[i]) + jointThomingOffsetRad[i];
    }

    return jointTrad;
}

void HAP_ARM::close(void)
{
    hapticDevice_assistoBot->close();
	hapticDevice_needleBot->close();
}

void HAP_ARM::timer_update(QTimerEvent *event)
{
	enable_spring_force_assistoBot_flag = enable_spring_force_assistoBot_button->isChecked();
	enable_chai_model_interaction_force_assistoBot_flag = enable_chai_model_interaction_force_assistoBot_button->isChecked();
	enable_ATI_force_assistoBot_flag = enable_ATI_force_assistoBot_button->isChecked();
	spring_force_gain_assistoBot = spring_force_gain_assistoBot_spinbox->getValue();
	ATI_gain_assistoBot = ATI_gain_assistoBot_spinbox->getValue();
	chai_model_interaction_gain_assistoBot = chai_model_interaction_gain_assistoBot_spinbox->getValue();

	enable_spring_force_needleBot_flag = enable_spring_force_needleBot_button->isChecked();
	enable_chai_model_interaction_force_needleBot_flag = enable_chai_model_interaction_force_needleBot_button->isChecked();
	enable_ATI_force_needleBot_flag = enable_ATI_force_needleBot_button->isChecked();
	spring_force_gain_needleBot = spring_force_gain_needleBot_spinbox->getValue();
	ATI_gain_needleBot = ATI_gain_needleBot_spinbox->getValue();
	chai_model_interaction_gain_needleBot = chai_model_interaction_gain_needleBot_spinbox->getValue();

    animation->updateGL();
    cVector3d newCameraPos = animation->R_cur*(animation->zoom*camera_pos_start);
    cVector3d newCameraUp = animation->R_cur*camera_up_start;
    cVector3d newCameraTarget = camera_target_start;
    camera->set(newCameraPos, newCameraTarget, newCameraUp);
    light->setPos(newCameraPos[0], newCameraPos[1], newCameraPos[2]);
	forces_text_label_assistoBot->setText("AssistoBot:  " +  QString(info_assistoBot.m_modelName.c_str()) + "<BR>Fx: " + QString::number(tool_assistoBot->m_lastComputedGlobalForce[0]) + "<BR>Fy: " + QString::number(tool_assistoBot->m_lastComputedGlobalForce[1]) + "<BR>Fz: " + QString::number(tool_assistoBot->m_lastComputedGlobalForce[2]));
	pos_text_label_assistoBot->setText("AssistoBot:" +  QString(info_assistoBot.m_modelName.c_str()) +  + "<BR>x: " + QString::number(tool_assistoBot->m_deviceGlobalPos.x) + "<BR>y: " + QString::number(tool_assistoBot->m_deviceGlobalPos.y) + "<BR>z: " + QString::number(tool_assistoBot->m_deviceGlobalPos.z));
	forces_text_label_needleBot->setText("NeedleBot:"  +  QString(info_needleBot.m_modelName.c_str()) + "<BR>Fx: " + QString::number(tool_needleBot->m_lastComputedGlobalForce[0]) + "<BR>Fy: " + QString::number(tool_needleBot->m_lastComputedGlobalForce[1]) + "<BR>Fz: " + QString::number(tool_needleBot->m_lastComputedGlobalForce[2]));
	pos_text_label_needleBot->setText("NeedleBot  " + QString(info_needleBot.m_modelName.c_str()) + "<br>x: " + QString::number(PosNeedleBot[0]) + "<br>y: " + QString::number(PosNeedleBot[1]) + "<br>z: " + QString::number(PosNeedleBot[2]) + "<br>yaw: " + QString::number(PosNeedleBot[3]) + "<br>pitch: " + QString::number(PosNeedleBot[4]) + "<br>roll: " + QString::number(PosNeedleBot[5]) + "<br>cathins: " + QString::number(PosNeedleBot[6]));

}

void HAP_ARM::change_enabling_all_forces_assistoBot(void)
{
		enable_spring_force_assistoBot_flag = 0;
		enable_chai_model_interaction_force_assistoBot_flag = 0;
		enable_ATI_force_assistoBot_flag = 0;

		enable_spring_force_assistoBot_button->setChecked(0);
		enable_chai_model_interaction_force_assistoBot_button->setChecked(0);
		enable_ATI_force_assistoBot_button->setChecked(0);

		tool_assistoBot->m_lastComputedGlobalForce[0] = 0;
		tool_assistoBot->m_lastComputedGlobalForce[1] = 0;
		tool_assistoBot->m_lastComputedGlobalForce[2] = 0;

		tool_assistoBot->applyForces();
}

void HAP_ARM::change_enabling_all_forces_needleBot(void)
{
		enable_spring_force_needleBot_flag = 0;
		enable_chai_model_interaction_force_needleBot_flag = 0;
		enable_ATI_force_needleBot_flag = 0;

		enable_spring_force_needleBot_button->setChecked(0);
		enable_chai_model_interaction_force_needleBot_button->setChecked(0);
		enable_ATI_force_needleBot_button->setChecked(0);

		tool_needleBot->m_lastComputedGlobalForce[0] = 0;
		tool_needleBot->m_lastComputedGlobalForce[1] = 0;
		tool_needleBot->m_lastComputedGlobalForce[2] = 0;

		tool_needleBot->applyForces();
}

void HAP_ARM::change_handedness(bool checked)
{
	left_handedness_flag = checked;
	if(checked == 0)
	{
		cout << "Changed haptic devices for right-handedness." << endl;
		left_handedness_button->setText("Right Handed");
	}
	else
	{
		cout << "Changed haptic devices for left-handedness." << endl;
		left_handedness_button->setText("Left Handed");
	}

	std::swap(tool_assistoBot, tool_needleBot);
}
