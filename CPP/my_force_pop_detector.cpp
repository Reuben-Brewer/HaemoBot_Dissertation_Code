#include "my_force_pop_detector.h"




my_force_pop_detector::my_force_pop_detector(QWidget *parent):
QWidget(parent)
{
   
	/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	background_box = new QLabel("",this);
	background_box->setGeometry(0,0,1300,260);
	background_box->setStyleSheet("*{border: 1px solid  #09A72E; border-radius: 10px;background-color: rgb(rgb(200,255,200))}");

	/////////////////////////background_box must be the first GUI object so that it doesn't cover over other objects!!!
	box_title = new QLabel("Force Pop Detector", this);
	box_title->setGeometry(5,5,150,30);
	box_title->setFont(QFont("Times", 12, QFont::Bold));

	///////////////////////////////////////////////////////
	reset_pop_detection_latch_button = new QPushButton("Reset Latch", this);
    reset_pop_detection_latch_button->setGeometry(QRect(5, 45, 120, 25));
    connect(reset_pop_detection_latch_button, SIGNAL(clicked()), this, SLOT(reset_pop_detection_latch(void)));
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	force_offset_snapshot_button = new QPushButton("Force Snapshot", this);
    force_offset_snapshot_button->setGeometry(QRect(5, 75, 120, 25));
    connect(force_offset_snapshot_button, SIGNAL(clicked()), this, SLOT(take_force_snapshot(void)));
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	int spinbox_start_x = 5;
	int spinbox_start_y = 105;
	int spinbox_start_width = 120;
	int spinbox_start_height = 30;
	int spinbox_start_height_inc = 5;

	force_pop_detection_threshold = -0.30;
	force_pop_detection_threshold_spinbox = new print_labeled_spinbox(this, "Fthresh: ", 0.0001, 10.0, -10.0, 1000.0, force_pop_detection_threshold);
	force_pop_detection_threshold_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 1*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));

	force_derivative_pop_detection_threshold = 1.3;
	force_derivative_pop_detection_threshold_spinbox = new print_labeled_spinbox(this, "FDERthresh: ", 0.0001, 10.0, -10.0, 1000.0, force_derivative_pop_detection_threshold);
	force_derivative_pop_detection_threshold_spinbox->setGeometry(QRect(spinbox_start_x, spinbox_start_y + 2*(spinbox_start_height + spinbox_start_height_inc), spinbox_start_width, spinbox_start_height));
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	int labels_start_x = 5;
	int labels_start_y = spinbox_start_y + 3*(spinbox_start_height + spinbox_start_height_inc);
	int labels_start_width = 150;
	int labels_start_height_single_line_text = 20;
	int labels_start_height_double_line_text = 35;
	int labels_start_height_inc = 5;

	force_at_last_pop_detection_label = new QLabel("", this);
	force_at_last_pop_detection_label->setGeometry(QRect(labels_start_x, labels_start_y + 0*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));

	force_derivative_at_last_pop_detection_label = new QLabel("", this);
	force_derivative_at_last_pop_detection_label->setGeometry(QRect(labels_start_x, labels_start_y + 1*(labels_start_height_single_line_text + labels_start_height_inc), labels_start_width, labels_start_height_single_line_text));
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	ATI_FT_in = std::vector<double>(6,0);
	last_ATI_FT_in = std::vector<double>(6,0);
    ATI_FT_derivative_raw = std::vector<double>(6,0);
	last_ATI_FT_derivative_raw = std::vector<double>(6,0);

	ATI_FT_in_OFFSET_BY_SNAPSHOT = std::vector<double>(6,0);
	last_ATI_FT_in_OFFSET_BY_SNAPSHOT = std::vector<double>(6,0);
	ATI_FT_derivative_raw_OFFSET_BY_SNAPSHOT = std::vector<double>(6,0);
	last_ATI_FT_derivative_raw_OFFSET_BY_SNAPSHOT = std::vector<double>(6,0);
	ATI_FT_in_SNAPSHOT = std::vector<double>(6,0);

	ATI_FT_derivative_smoothed = std::vector<double>(6,0);
	pop_detection_state = 0;
	pop_detection_state_latched = 0;
	force_at_last_pop_detection = 0;
	force_derivative_at_last_pop_detection = 0;
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
    plot_mutex = new QMutex;

    int plot_height = 175;
	int plot_height_inc = 5;
    int plot_width = 370;
	int plot_width_inc = 5;
	int plot_start_x = 150;
	int plot_start_y = 5;

    std::vector<double *> temp_0;
    temp_0.push_back(&ATI_FT_in_OFFSET_BY_SNAPSHOT[2]);
    temp_0.push_back(&force_pop_detection_threshold);
    plot_data_vec.push_back(temp_0);
    plot.push_back(new DataPlot(plot_data_vec[0], 1, -8.0, 8.0, plot_height, QString("F (N)"), "N", 0, this, plot_mutex));
    plot[0]->setGeometry(QRect(plot_start_x + 0*(plot_width + plot_width_inc), plot_start_y + 0*(plot_height + plot_height_inc), plot_width, plot_height));

    std::vector<double *> temp_1;
    temp_1.push_back(&ATI_FT_derivative_smoothed[2]);
    temp_1.push_back(&force_derivative_pop_detection_threshold);
    plot_data_vec.push_back(temp_1);
    plot.push_back(new DataPlot(plot_data_vec[1], 1, -8.0, 8.0, plot_height, QString("F Der (N/s)"), "N/s", 0, this, plot_mutex));
    plot[1]->setGeometry(QRect(plot_start_x + 1*(plot_width + plot_width_inc), plot_start_y + 0*(plot_height + plot_height_inc), plot_width, plot_height));

    std::vector<double *> temp_2;
    temp_2.push_back(&pop_detection_state);
    temp_2.push_back(&pop_detection_state_latched);
    plot_data_vec.push_back(temp_2);
    plot.push_back(new DataPlot(plot_data_vec[2], 1, 0, 2.0, plot_height, QString("Pop State"), "Bool", 0, this, plot_mutex));
    plot[2]->setGeometry(QRect(plot_start_x + 2*(plot_width + plot_width_inc), plot_start_y + 0*(plot_height + plot_height_inc), plot_width, plot_height));
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	force_derivative_bessel_filter = new bessel_filter(this, 6); 
	///////////////////////////////////////////////////////
}


void my_force_pop_detector::update(std::vector<double> force_vector_in, double servo_loop_frequency_set_by_slider)
{
	
	for(int i = 0; i < force_vector_in.size(); i++)
	{
		ATI_FT_in[i] = force_vector_in[i];

		ATI_FT_in_OFFSET_BY_SNAPSHOT[i] = ATI_FT_in[i] - ATI_FT_in_SNAPSHOT[i];

		ATI_FT_derivative_raw[i] = (ATI_FT_in[i]-last_ATI_FT_in[i])*(1.0/(1.0/servo_loop_frequency_set_by_slider));	
		ATI_FT_derivative_raw_OFFSET_BY_SNAPSHOT[i] = (ATI_FT_in_OFFSET_BY_SNAPSHOT[i]-last_ATI_FT_in_OFFSET_BY_SNAPSHOT[i])*(1.0/(1.0/servo_loop_frequency_set_by_slider));	
		
		last_ATI_FT_in = ATI_FT_in;
		last_ATI_FT_in_OFFSET_BY_SNAPSHOT = ATI_FT_in_OFFSET_BY_SNAPSHOT;
		
		last_ATI_FT_derivative_raw[i] = ATI_FT_derivative_raw[i];
		last_ATI_FT_derivative_raw_OFFSET_BY_SNAPSHOT[i] = ATI_FT_derivative_raw_OFFSET_BY_SNAPSHOT[i];
	}

	ATI_FT_derivative_smoothed = force_derivative_bessel_filter->update(ATI_FT_derivative_raw_OFFSET_BY_SNAPSHOT);

	double current_pop_force = ATI_FT_in_OFFSET_BY_SNAPSHOT[2];
	double current_pop_force_derivative = ATI_FT_derivative_smoothed[2];
	if(current_pop_force <= force_pop_detection_threshold && current_pop_force_derivative >= force_derivative_pop_detection_threshold)
	{
		pop_detection_state = 1;
		pop_detection_state_latched = 1;

		force_at_last_pop_detection = current_pop_force;
		force_derivative_at_last_pop_detection = current_pop_force_derivative;
	}
	else
	{
		pop_detection_state = 0;
	}

}

void my_force_pop_detector::close(void)
{
    
}

void my_force_pop_detector::timer_update(QTimerEvent *event)
{

	force_pop_detection_threshold = force_pop_detection_threshold_spinbox->getValue();
	force_derivative_pop_detection_threshold = force_derivative_pop_detection_threshold_spinbox->getValue();

	force_at_last_pop_detection_label->setText("Last PopForce: " + QString::number(force_at_last_pop_detection));
	force_derivative_at_last_pop_detection_label->setText("Last PopForceDER: " + QString::number(force_derivative_at_last_pop_detection));

    for(int i = 0; i < plot.size(); i++)
    {
        plot[i]->update_data_plot(event);
    }
}

void my_force_pop_detector::reset_pop_detection_latch(void)
{
	pop_detection_state_latched = 0;
	force_at_last_pop_detection = 0;
	force_derivative_at_last_pop_detection = 0;

	cout << "Reset the pop detection latch." << endl;
}


void my_force_pop_detector::take_force_snapshot(void)
{
	ATI_FT_in_SNAPSHOT = ATI_FT_in;
	cout << "Took force snapshot" << endl;
}