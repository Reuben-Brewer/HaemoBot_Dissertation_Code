#include "print_labeled_spinbox.h"


print_labeled_spinbox::print_labeled_spinbox(QWidget *parent, QString pre_text_in, float single_step_in, int decimal_precision_in, float min_range_in, float max_range_in, float init_value_in):
    QWidget(parent)
{
	
   this->pre_text=pre_text_in;
   this->single_step=single_step_in;
   this->decimal_precision=decimal_precision_in;
   this->min_range=min_range_in;
   this->max_range=max_range_in;
   this->init_value=init_value_in;

	current_value = init_value;

   d_label = new QLabel(pre_text, this);

   QFont d_label_font = d_label->font();
   d_label_font.setPointSize(8);
   d_label_font.setBold(false);
   d_label->setFont(d_label_font);

   QFontMetrics d_label_font_metrics(d_label_font);
   int text_width=d_label_font_metrics.width(pre_text);

   //QRect pos_d_label = QRect(0, 0, 5.0*int(pre_text.length()), 20);
   QRect pos_d_label = QRect(0, 0, text_width, 20);
   d_label->setGeometry(pos_d_label);



   //int text_width = d_label->width();

   my_spin_box = new QDoubleSpinBox(this);  
   QRect pos_my_spin_box = QRect(text_width+0, 2, 60, 20);   
   my_spin_box->setDecimals(decimal_precision); //this line must be set first so as not to round any of the following parameters!
   my_spin_box->setGeometry(pos_my_spin_box);
   my_spin_box->setSingleStep(single_step);
   my_spin_box->setRange(min_range,max_range);
   my_spin_box->setValue(init_value);
   my_spin_box->setKeyboardTracking(0); //SO THAT THE valueChanged() SLOT WON'T WIRE UNTIL ENTER IS PRESSED, SPINBOX LOSES FOCUS, OR THE UP/DOWN ARROWS ARE PRESSED.

   connect(my_spin_box, SIGNAL(valueChanged(double)), this, SLOT(editingFinishedResponseFunction(double)));
}

double print_labeled_spinbox::getValue(void)
{
	//return my_spin_box->value();
	return current_value;
}

void print_labeled_spinbox::setValue(double val)
{
	my_spin_box->setValue(val);
}

void print_labeled_spinbox::enable(void)
{
	my_spin_box->setEnabled(1);
	//printf("enabled \n");
}

void print_labeled_spinbox::disable(void)
{
	my_spin_box->setEnabled(0);
	//printf("disabled \n");
}

void print_labeled_spinbox::keyPressEvent(QKeyEvent *incoming_key_event) //Function must be named "keyPressEvent" to properly overide the normal even handler and operate correctly.
{
	incoming_key_event->accept();
	QString val = incoming_key_event->text();
	//std::cout << "Key event in print_labeled_spinbox: " << val.toStdString() << std::endl;

	incoming_key_event->ignore();
	QWidget::keyPressEvent(incoming_key_event); //This line is recommended by the QT class documentation.
}

void print_labeled_spinbox::editingFinishedResponseFunction(double val)
{
	current_value = val;
	//cout << "spinbox returned with a value of: " << current_value << endl;
}