#include "my_QGLWidget.h"
#include <QMouseEvent>
#include "robot_defines.h"
#include "chai3d.h"



my_QGLWidget::my_QGLWidget(cCamera *camera_in, QWidget *parent, const my_QGLWidget *shareWidget, string label_text): QGLWidget(parent, shareWidget)
{
    camera = camera_in;

    zoom = 1;

    Rz.identity();
    Rx.identity();
    R_cur.identity();

    label = new QLabel(this);
    label->setText(QString(label_text.c_str()));
    label->setFont(QFont("Times", 24, QFont::Bold));
    label->setGeometry(QRect(200,0,410, 50));
    label->setStyleSheet("*{background-color: rgb(200,200,200)}");
}


void my_QGLWidget::mousePressEvent(QMouseEvent *event) //Function is necessary for keeping track of where we last pressed in the window
{
    last_mouse_pos[0] = event->QMouseEvent::x();
    last_mouse_pos[1] = event->QMouseEvent::y();
}

void my_QGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    mouse_pos[0] = event->QMouseEvent::x();
    mouse_pos[1] = event->QMouseEvent::y();

    Qt::MouseButtons buttons = event->buttons();
    if(buttons == Qt::LeftButton)
    {
        //printf("Left Click \n");
        float rot_gain = 0.5;
        Rz = cRotMatrix(cVector3d(0.0, 0.0, 1.0), (pi_define/180)*rot_gain*(mouse_pos[0] - last_mouse_pos[0]));
        Rx = cRotMatrix(cVector3d(1.0, 0.0, 0.0), (pi_define/180)*rot_gain*(mouse_pos[1] - last_mouse_pos[1]));

        R_cur = Rz*Rx*R_cur;
    }
    else if(buttons == Qt::RightButton)
    {
        
        double zoom_gain = 0.05;
        zoom = zoom + zoom_gain*(mouse_pos[1] - last_mouse_pos[1]);
        //printf("Right Click, Zoom Factor %lf\n", zoom);
    }

    last_mouse_pos = mouse_pos;
}

void my_QGLWidget::intializeGL(void)
{

}

void my_QGLWidget::paintGL(void)
{
    camera->renderView(width(), height());
}

void my_QGLWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);
}



