#include "my_data_log.h"

my_data_log::my_data_log(QWidget *parent):
    QWidget(parent)
{

	/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,380,150);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

	box_title = new QLabel("Data Logger",this);
	box_title->setGeometry(10,10,150,30);
	box_title->setFont(QFont("Times", 12, QFont::Bold));

	record_state_button = new QPushButton("Start recording.", this);
    record_state_button->setGeometry(QRect(10, 50, 100, 20));
    connect(record_state_button, SIGNAL(clicked()), this, SLOT(change_record_state_button_toggle()));
	save_data_state = 0;
	record_state_button->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(200,255,200)}");

	file_name_input_box = new QLineEdit(this);
    file_name_input_box->setGeometry(QRect(120, 50, 160, 20));
    file_name_input_box->setText("Filename");
    file_name_input_box->show();
	//connect(file_name_input_box, SIGNAL(editingFinished()), this, SLOT(editingFinishedResponseFunction(void)));
	connect(file_name_input_box, SIGNAL(returnPressed()), this, SLOT(editingFinishedResponseFunction(void)));

	double frequency_max = 1000;
	slider_log_frequency = new slider_with_box(this, "Log Freq", 0, 1, frequency_max, 1, frequency_max);
    slider_log_frequency->setGeometry(QRect(10, 80, 450, 60));
    //connect(slider_log_frequency, SIGNAL(valueChanged(double)), this, SLOT(slider_log_frequency_changed(void)));
	desired_data_point_log_frequency = 1; //Minimum allowable frequency as the default.

	this->setFocusPolicy(Qt::ClickFocus);

	focus_needs_to_be_restored_to_other_widgets_flag = 1;
}

void my_data_log::editingFinishedResponseFunction(void)
{
	cout << "editing finished" << endl;
	this->clearFocus(); //Takes keyboard focus away from this widget so that our USB footpedal object will respond correctly again.
	file_name_input_box->clearFocus(); 

	focus_needs_to_be_restored_to_other_widgets_flag = 1;
}

void my_data_log::mousePressEvent(QMouseEvent *incoming_mouse_event)
{
	cout << "my_data_log mouse event" << endl;
}

void my_data_log::slider_log_frequency_changed(void)
{
	
}


void my_data_log::timer_update(QTimerEvent *event)
{
	file_name_input = file_name_input_box->text();
	desired_data_point_log_frequency = slider_log_frequency->getSliderVal();

	if(save_data_state == 1)
	{
		record_state_button->setText("Stop recording.");
		record_state_button->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(255,200,200)}");
		file_name_input_box->setEnabled(0);
		slider_log_frequency->disable();
	}
	else if(save_data_state == 0)
	{
		record_state_button->setText("Start recording");
		record_state_button->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(200,255,200)}");
		file_name_input_box->setEnabled(1);
		slider_log_frequency->enable();
	}
}

void my_data_log::change_record_state_button_toggle(void)
{
	if(save_data_state == 0)
	{
		actual_file_name_to_save_with = file_name_input; //If the user pressed the button, then just go with the name they entered.
		save_data_state = 1;
		change_record_state(save_data_state);
	}
	else if(save_data_state == 1)
	{
		save_data_state = 0;
		change_record_state(save_data_state);
	}
}

void my_data_log::change_record_state(bool val)
{
	if(val == 1)
	{
		save_data_state = 1;
		create_and_open_data_file();
		cout << "opening data log file" << endl;
	}
	else if(val == 0)
	{
		save_data_state = 0;
		save_data();
		cout << "closing/saving data log file" << endl;
	}
}

bool my_data_log::check_if_time_to_add_another_data_point(double curTime)
{
	double desiredDeltaT = 1.0/desired_data_point_log_frequency;

	if((curTime - time_last_data_point_added) >= desiredDeltaT)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void my_data_log::create_and_open_data_file(void)
{
	char timestamp[] = "_07_30_2009_03_04_30";
	char data_file_name[100];
	char file_dir[] = "C:/robot_generated_data/";
	char dot_file_extension[] = ".dat";

	time_t rawtime;
	struct tm * timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime (timestamp,strlen(timestamp)+1,"_%m_%d_%Y_%I_%M_%S", timeinfo); //In strlen(timestamp)+1, need the +1 to account for the added NULL-terminator.
	sprintf(data_file_name, "%s%s%s%s",file_dir, actual_file_name_to_save_with.toStdString().c_str(), timestamp, dot_file_extension);

	cout << "Data file name: " << data_file_name << endl;

	data_file = fopen(data_file_name,"w");
	data_to_write.clear();

	save_data_state = 1;

	/*//std::string system_call_string =  "\"\"C:\\Program Files\\VideoLAN\\VLC\\vlc.exe\" -vvv dshow:// :dshow-vdev=\"Sensoray 811 BDA Analog Capture\" --sout=#transcode{vcodec=h264,acodec=mpga,ab=128,channels=2,samplerate=44100}:file{dst=C:\\Users\\reuben\\Desktop\\test2.mp4,no-overwrite} :sout-all :sout-keep";
	std::string system_call_string =  "\"C:\\VLC_stream.bat &"; 
	cout << system_call_string << endl;
	int foo = system("start");
	int dummy = system(system_call_string.c_str());*/
}

void my_data_log::save_data()
{
	fprintf(data_file, "current_global_time, encoder_vec_0, encoder_vec_1, encoder_vec_2, encoder_vec_3, encoder_vec_4, encoder_vec_5, encoder_vec_6, encoder_dot_vec_bessel_0, encoder_dot_vec_bessel_1, encoder_dot_vec_bessel_2, encoder_dot_vec_bessel_3, encoder_dot_vec_bessel_4, encoder_dot_vec_bessel_5, encoder_dot_vec_bessel_6, NBcont_NB_PosDes_0, NBcont_NB_PosDes_1, NBcont_NB_PosDes_2, NBcont_NB_PosDes_3, NBcont_NB_PosDes_4, NBcont_NB_PosDes_5, NBcont_NB_PosDes_6, NBcont_assistobotPosDes_0, NBcont_assistobotPosDes_1, NBcont_assistobotPosDes_2, ATI_FT_smoothed_assistoBot_0, ATI_FT_smoothed_assistoBot_1, ATI_FT_smoothed_assistoBot_2, ATI_FT_smoothed_assistoBot_3, ATI_FT_smoothed_assistoBot_4, ATI_FT_smoothed_assistoBot_5, ATI_FT_smoothed_needleBot_0, ATI_FT_smoothed_needleBot_1, ATI_FT_smoothed_needleBot_2, ATI_FT_smoothed_needleBot_3, ATI_FT_smoothed_needleBot_4, ATI_FT_smoothed_needleBot_5, force_pop_detection_threshold, force_derivative_pop_detection_threshold, pop_detection_state, pop_detection_state_latched, force_pop_detector_ATI_FT_derivative_smoothed_0, force_pop_detector_ATI_FT_derivative_smoothed_1, force_pop_detector_ATI_FT_derivative_smoothed_2, force_pop_detector_ATI_FT_derivative_smoothed_3, force_pop_detector_ATI_FT_derivative_smoothed_4, force_pop_detector_ATI_FT_derivative_smoothed_5, force_pop_detector_ATI_FT_in_OFFSET_BY_SNAPSHOT_0, force_pop_detector_ATI_FT_in_OFFSET_BY_SNAPSHOT_1, force_pop_detector_ATI_FT_in_OFFSET_BY_SNAPSHOT_2, force_pop_detector_ATI_FT_in_OFFSET_BY_SNAPSHOT_3, force_pop_detector_ATI_FT_in_OFFSET_BY_SNAPSHOT_4, force_pop_detector_ATI_FT_in_OFFSET_BY_SNAPSHOT_5, auto_insertion_depth_before_detecting_pop, auto_current_insertion_depth, ND_accel_0, ND_accel_1, ND_accel_2, assistoBot_EE_0, assistoBot_EE_1, assistoBot_EE_2, auto_insertion_substate\n"); //comma delimited

	for(int counter = 0; counter < data_to_write.size(); counter++)
	{
		fprintf(data_file, "%s\n", data_to_write[counter].c_str());
	}

	fclose(data_file);

	save_data_state = 0; //ONLY SAY WE'RE NOT SAVING ONCE WE'RE REALLY DONE.
}


