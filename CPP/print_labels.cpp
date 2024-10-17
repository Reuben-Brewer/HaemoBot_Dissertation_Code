#include <math.h>
#include <qapplication.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qwt_slider.h>
#include <qwt_scale_engine.h>
#include <qwt_scale_map.h>
#include "print_labels.h"

print_labels::print_labels(QWidget *parent, float* label_num_in, QString pre_text_in, int decimal_precision_in):
    QWidget(parent)
{
	
   this->label_num=label_num_in;
   this->pre_text=pre_text_in;
   this->decimal_precision = decimal_precision_in;
   
   QString dummy = "";
   int i = 0;
   for(int i = 0; i < 6+int(pre_text.length()) ; i++)
   {
	dummy.append("O");
   }

   //printf("%s \n", dummy.data()); //dummy.toAscii().constData()


   d_label = new QLabel(dummy, this); //new is for pointers only
    //d_label->setAlignment(Qt::AlignTop);
    //d_label->setFixedWidth(d_label->fontMetrics().width("12345"));
	//d_label->setMargin(10);
}


void print_labels::update_label(QTimerEvent *)
{
	QString text;
	QString pre_text_copy;

	pre_text_copy = pre_text;
        text.setNum(*label_num, 'f', decimal_precision);
	pre_text_copy.append(text);

        d_label->setText(pre_text_copy);


    // update the display
    //replot();
}
