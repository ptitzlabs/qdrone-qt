#ifndef GL_WIDGET_H
#define GL_WIDGET_H


#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <QObject>
#include <QWidget>
#include <QGLWidget>


#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
//#include <freetype2/config/ftheader.h>
//#include <freetype2/ft2build.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <cstring>

class gl_widget : public QGLWidget
{
    Q_OBJECT
public:
    explicit gl_widget(QWidget *parent = 0);
    void resizeGL(int w, int h);
    void initializeGL();
    void paintGL();
        void refresh_instruments(double phi,
                                 double the,
                                 double z,
                                 double zd,
                                 double xd,
                                 double yd,
                                 double psi,
                                 QString * control_name);

private:
    int width; // screen width in px
    int height; // screen width in px
    double FOV_v; // vertical field of view
    double FOV_h; // horizontal field of view

    double tan_FOV_v;
    double tan_FOV_h;

    double _phi; // roll
    double _the; // pitch
    double _z;
    double _zd;
    double _xd;
    double _yd;
    double _psi;
    double _phi_ticks_xy_in[21][2];
    double _phi_ticks_xy_out[21][2];
    double _phi_ticks_r;
    double _the_ticks;
    int _n_the_ticks;

    double _z_ticks;
    int _n_z_ticks;
    double _z_ticks_incr;

    double _vel_ticks;
    int _n_vel_ticks;
    double _vel_ticks_incr;
    double _vel;

    double _psi_ticks;
    int _n_psi_ticks;
    double _psi_ticks_incr;


    double _ver_offset; // theta offset

    QString * _control_name;

    void calc_offset(); // calculating horizon offset

    void paint_roll_ticks();


    void draw_text();


};

#endif // GL_WIDGET_H
