#ifndef HAP_ARM_H
#define HAP_ARM_H

#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <QMutex>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include "chai3d.h"
#include "chaiHelperFunctions.h"
#include "robot_defines.h"
#include "my_QGLWidget.h"
#include <cml/cml.h>
#include "print_labeled_spinbox.h"

using namespace std;

class HAP_ARM: public QWidget
{
    Q_OBJECT

public:
    HAP_ARM(QWidget *parent = NULL);
    void timer_update(QTimerEvent *e);
    void updateAndApplyForce(cVector3d commandedPos_assistoBot, cVector3d actualPos_assistoBot, std::vector<double> ATI_assistoBot_force_vec_in, cVector3d commandedPos_needleBot, cVector3d actualPos_needleBot, std::vector<double> ATI_needleBot_force_vec_in);
    void close(void);
    void DisplayKinematics(cVector3d jointTrad, cVector3d EndEff);
    void updateHapticPose(void);
    cVector3d convertAngle2steps(cVector3d jointTrad);
    cVector3d convertSteps2angle(cVector3d jointTsteps);
    cVector3d FwdKinematics(cVector3d jointTrad);
    cVector3d InvKinematics(cVector3d EndEff);

	QRadioButton *left_handedness_button;
	int left_handedness_flag;
    QPushButton *enable_all_forces_assistoBot_button;
	QRadioButton *enable_spring_force_assistoBot_button, *enable_chai_model_interaction_force_assistoBot_button, *enable_ATI_force_assistoBot_button;
	print_labeled_spinbox *spring_force_gain_assistoBot_spinbox, *ATI_gain_assistoBot_spinbox, *chai_model_interaction_gain_assistoBot_spinbox;
	double spring_force_gain_assistoBot, ATI_gain_assistoBot, chai_model_interaction_gain_assistoBot;
	bool enable_all_forces_assistoBot_flag, enable_spring_force_assistoBot_flag, enable_chai_model_interaction_force_assistoBot_flag, enable_ATI_force_assistoBot_flag;
	cVector3d spring_force_assistoBot;

	QPushButton *enable_all_forces_needleBot_button;
	QRadioButton *enable_spring_force_needleBot_button, *enable_chai_model_interaction_force_needleBot_button, *enable_ATI_force_needleBot_button;
	print_labeled_spinbox *spring_force_gain_needleBot_spinbox, *ATI_gain_needleBot_spinbox, *chai_model_interaction_gain_needleBot_spinbox;
	double spring_force_gain_needleBot, ATI_gain_needleBot, chai_model_interaction_gain_needleBot;
	bool enable_all_forces_needleBot_flag, enable_spring_force_needleBot_flag, enable_chai_model_interaction_force_needleBot_flag, enable_ATI_force_needleBot_flag;
	cVector3d spring_force_needleBot;

	QLabel *forces_text_label_assistoBot, *forces_text_label_needleBot, *pos_text_label_assistoBot, *pos_text_label_needleBot, *background_box, *box_title;
    my_QGLWidget *animation;
    int animation_width, animation_height, animation_x, animation_y;

    cHapticDeviceHandler *handler;
    cGenericHapticDevice *hapticDevice_assistoBot, *hapticDevice_needleBot;
    cGeneric3dofPointer *tool_assistoBot, *tool_needleBot;
	cHapticDeviceInfo info_assistoBot, info_needleBot;
    cWorld *world;
    cCamera *camera;
    cLight *light;
    cMaterial *my_material;
    cShapeLine *force_chaiFrame_line_1;
    std::vector<cMesh *> cmesh_model_vec;
    std::vector<string> cmesh_model_names;
    cVector3d camera_pos_start, camera_target_start, camera_up_start;
	cVector3d model_center;

    cVector3d toolPos_assistoBot, toolVel_assistoBot, toolPos_needleBot, toolVel_needleBot;
	cMatrix3d toolRot_assistoBot, toolRot_needleBot;

    cVector3d objectPos;
    cVector3d write_omni_force;
    cVector3d EE, joint_vector_rad, joint_vector_num_full_steps, stepper_gear_ratio_rad_to_full_step_vec, assistoBot_central_axis, jointThomingOffsetRad;
    cMatrix3d toolRot, robot_rot_mat_chai, tracker_rot_mat_chai;
    double L1, L2, L3, L4;

	int button_0_state_needleBot, button_1_state_needleBot, button_0_state_assistoBot, button_1_state_assistoBot;
	std::vector<double> PosNeedleBot, VelNeedleBot, PosAssistoBot;
	cml::matrix33f_c rot_matrix_CML_needleBot;

public Q_SLOTS:
	void change_enabling_all_forces_assistoBot(void);
    void change_enabling_all_forces_needleBot(void);
	void change_handedness(bool checked);

};



#endif





