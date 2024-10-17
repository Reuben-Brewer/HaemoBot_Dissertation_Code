#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qapplication.h>
#include <qmainwindow.h>
#include <qwt_counter.h>
#include <qtoolbar.h>
#include <qlabel.h>
#include <qlayout.h>
#include <QMutex>
#include <QSize>
#include <QThread>
#include <QWaitCondition>
#include <QApplication>
#include <QFont>
#include <QLCDNumber>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QWidget>
#include <QDialog>
#include<QGroupBox>
#include<QFormLayout>
#include<QRadioButton>
#include<QSpinBox>
#include<QImage>
#include<QImageReader>
#include<QPixmap>
#include<QPalette>
#include<QObject>
#include <QPainter>
#include <QMessageBox>
#include <QLineEdit>
#include <QComboBox>
#include <QLabel>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include "slider_with_box.h"
#include "my_needle_driver_programmer.h"
#include "chai3d.h"
#include <cml/cml.h>
#include "HAP_ARM.h"
#include "my_stepper.h"
#include "my_stepper_programmer.h"
#include "daq_boards.h"
#include "ATI_force_sensor.h"
#include "print_motor_info.h"
#include "needleBot_controller.h"
#include "robot_defines.h"
#include "my_data_log.h"
#include "dynamixel_servo.h"
#include "dynamixel_servo_programmer.h"
#include "BP_cuff.h"
#include "data_plot.h"
#include "bessel_filter.h"
#include "my_stereo_camera_electronics_controller_QT.h"
#include "my_LED_board_programmer.h"
#include "my_force_pop_detector.h"
#include "my_foot_pedal_USB_stinky.h"
#include <QKeyEvent>

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    void timerEvent(QTimerEvent *event);
	void keyPressEvent(QKeyEvent *event);

    int gui_timer_id;
    int start_or_end_program_flag;
    QPushButton *end_program_button;
	QLabel *mainwindow_current_global_time_label, *mainwindow_debug_label;
	double mainwindow_debug_var_0, mainwindow_debug_var_1;

	my_foot_pedal_USB_stinky *foot_pedal_USB_stinky;
	my_force_pop_detector *force_pop_detector;
    my_stereo_camera_electronics_controller_QT *stereo_camera_electronics_controller_QT;
	my_LED_board_programmer *LED_board_programmer;
	my_stepper *stepper_0, *stepper_1, *stepper_2;
    my_stepper_programmer *stepper_programmer;
    my_needle_driver_programmer *needle_driver_programmer;
	DAQ_board *my_DAQ;
	ATI_force_sensor *ATI_assistoBot, *ATI_needleBot;
    HAP_ARM *M_HAP_ARM;
	std::vector<print_motor_info *> motor_GUI_vec;
	needleBot_controller *needleBot_controller_instance;
	my_data_log *data_log;
	dynamixel_servo *servo_2_ethanol, *servo_3_air, *servo_4_restraint;
    dynamixel_servo_programmer *servo_programmer;
	BP_cuff *my_BP_cuff;
	slider_with_box *servo_loop_frequency_slider;
	double servo_loop_frequency;
	double mainWindow_gui_start_x_top_monitor, mainWindow_gui_start_y_top_monitor, mainWindow_gui_start_x_bottom_monitor, mainWindow_gui_start_y_bottom_monitor, mainWindow_gui_single_monitor_width, mainWindow_gui_single_monitor_height;
	

	/////////////////////////  Variables for the debug plot where we plot whatever variable we need to visualize for debugging
	double debug_plot_min_val, debug_plot_max_val;
	std::vector<double *> debug_plot_data_vec;
	DataPlot *debug_plot;
	QMutex *debug_plot_mutex;
	/////////////////////////

	/////////////////////////
	bessel_filter *velocity_bessel_filter;
	/////////////////////////

    QMutex *stepper_Rx_mutex, *needle_driver_Rx_mutex; 
    ~MainWindow();
    



public Q_SLOTS:
        void end_program();
        void start_program();


};

#endif
