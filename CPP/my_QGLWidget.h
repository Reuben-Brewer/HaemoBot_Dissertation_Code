#ifndef _MY_QGLWIDGET_H
#define _MY_QGLWIDGET_H 1

#include <QGLWidget>
#include <QLabel>
#include "chai3d.h"

class my_QGLWidget: public QGLWidget
{
    Q_OBJECT

public:

    my_QGLWidget(cCamera *camera_in = NULL, QWidget *parent = 0, const my_QGLWidget *shareWidget = 0, string label_text = "");
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

    cCamera *camera;
    cMatrix3d Rz, Rx, R_cur;
    cVector3d last_mouse_pos, mouse_pos;
    QLabel *label;
    double zoom;

public Q_SLOTS:

protected:
    virtual void intializeGL(void);
    virtual void paintGL(void);
    virtual void resizeGL(int width, int height);

private:




};

#endif
