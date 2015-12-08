#include "gl_widget.h"

gl_widget::gl_widget(QWidget *parent):
    QGLWidget(parent),
    FOV_v(M_PI/2),
    FOV_h(M_PI/2),
  _phi(0),
  _the(0)
{
    tan_FOV_v = tan(FOV_v/2);
    tan_FOV_h = tan(FOV_h/2);
    calc_offset();

    double phi_ticks_cos;
    double phi_ticks_sin;
    double phi_ticks_incr = M_PI/30;
    _phi_ticks_r = 0.5;
    double phi_ticks_angle;
    for (int i = 0; i <= 20; i++){
        phi_ticks_angle = M_PI/6 + (double)i*phi_ticks_incr;
        phi_ticks_cos = cos(phi_ticks_angle);
        phi_ticks_sin = sin(phi_ticks_angle);
        // outer tick coordinates

        if(i%5 == 0){
        _phi_ticks_xy_in[i][0] = (_phi_ticks_r-0.07)*phi_ticks_cos;
        _phi_ticks_xy_in[i][1] = (_phi_ticks_r-0.07)*phi_ticks_sin;
        } else{
            _phi_ticks_xy_in[i][0] = (_phi_ticks_r-0.05)*phi_ticks_cos;
            _phi_ticks_xy_in[i][1] = (_phi_ticks_r-0.05)*phi_ticks_sin;
        }
        // inner tick coordinates
        _phi_ticks_xy_out[i][0] = (_phi_ticks_r)*phi_ticks_cos;
        _phi_ticks_xy_out[i][1] = (_phi_ticks_r)*phi_ticks_sin;

    }

    _the_ticks = tan(M_PI/18)/tan_FOV_v;

    _n_the_ticks = floor((2*_phi_ticks_r-0.2)/_the_ticks)+1;

    _z_ticks = 0.12;
    _n_z_ticks = floor((2*_phi_ticks_r)/_z_ticks)+1;
    _z_ticks_incr = 0.5;

    _vel_ticks = _z_ticks;
    _n_vel_ticks = _n_z_ticks;
    _vel_ticks_incr = 0.5;

    _psi_ticks = _z_ticks;
    _n_psi_ticks = floor((2*_phi_ticks_r)*0.5/_z_ticks)+1;
    _psi_ticks_incr = 15;

}

void gl_widget::resizeGL(int w, int h){
//    int side = qMin(width, height);
    width = w;
    height = h;
    glViewport(0,0,width,height);
    std::cout<< "lookie! \n";

}

void gl_widget::initializeGL(){
//    qglClearColor(QColor(Qt::black));

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glEnable(GL_POLYGON_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0,0,0,0);

}

void gl_widget::calc_offset(){
    double tan_the = tan(_the);
    _ver_offset = -tan_the/tan_FOV_v;
}

void gl_widget::refresh_instruments(double phi,
                                    double the,
                                    double z,
                                    double zd,
                                    double xd,
                                    double yd,
                                    double psi){
    _phi = phi;
    _the = the;
    _psi = psi;
    _z = z;
    _zd = zd;
    _xd = xd;
    _yd = yd;
    _vel = sqrt(_xd*_xd+_yd*_yd);
//    std::cout<<"phi: "<<_phi<< " the: " << _the << std::endl;

    calc_offset();
//    initializeGL();
//    paintGL();
}

void gl_widget::paintGL(){
//    glViewport(0,0,200,200);
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
glLoadIdentity();


glRotatef(_phi*180/M_PI,0,0,1);
//glRotatef(_the*180/M_PI,1,0,0);
glTranslatef(0.0f,_ver_offset,0.0f);

glColor3ub(123, 74, 18 );
// brown
glBegin(GL_QUADS);
glVertex2f(-100.0f,0);
glVertex2f(100.0f,0);
glVertex2f(100.0f,-100.0f);
glVertex2f(-100.0f,-100.0f);
glEnd();


// blue
//glClear(GL_COLOR_BUFFER_BIT);
glColor3ub(115, 143, 155 );
//glColor3f(0,0,1);

glBegin(GL_QUADS);
glVertex2f(-100.0f,0);
glVertex2f(100.0f,0);
glVertex2f(100.0f,100.0f);
glVertex2f(-100.0f,100.0f);
glEnd();

glColor3ub(239, 221, 111  );
glBegin(GL_LINES);
glVertex2f(-100, 0);
glVertex2f(100, 0);
glEnd();

//glRotatef(_the*180/M_PI,1,0,0);
glTranslatef(0.0f,-_ver_offset,0.0f);
glRotatef(-_phi*180/M_PI,0,0,1);

glLineWidth(2.5);
glColor3f(1.0, 1.0, 1.0);
glBegin(GL_LINES);
glVertex2f(-_phi_ticks_r, 0.0f);
glVertex2f(-_phi_ticks_r+0.2, 0.0f);
glEnd();
glBegin(GL_LINES);
glVertex2f(-_phi_ticks_r+0.2, 0.0f);
glVertex2f(-_phi_ticks_r+0.2, -0.05f);
glEnd();
glBegin(GL_LINES);
glVertex2f(_phi_ticks_r-0.2, -0.05f);
glVertex2f(_phi_ticks_r-0.2, 0.0f);
glEnd();
glBegin(GL_LINES);
glVertex2f(_phi_ticks_r-0.2, 0.0f);
glVertex2f(_phi_ticks_r, 0.0f);
glEnd();

paint_roll_ticks();

}

void gl_widget::paint_roll_ticks(){

    glColor3f(1.0, 1.0, 1.0);
    glLineWidth(1.0);
    for (int i = 0; i <=20; i++) {
        glBegin(GL_LINES);
        glVertex2f(_phi_ticks_xy_in[i][0], _phi_ticks_xy_in[i][1]);
        glVertex2f(_phi_ticks_xy_out[i][0], _phi_ticks_xy_out[i][1]);
        glEnd();
    }

    glRotatef(_phi*180/M_PI,0,0,1);

    glBegin(GL_LINES);
    glVertex2f(0, _phi_ticks_r-0.1);
    glVertex2f(0, -_phi_ticks_r+0.1);
    glEnd();

    int top_the_tick_index = floor((_phi_ticks_r-0.1-_ver_offset)/_the_ticks);

    for (int i = 0; i < _n_the_ticks; i++){
        glBegin(GL_LINES);
        glVertex2f(-0.1, _ver_offset+(top_the_tick_index-i)*_the_ticks);
        glVertex2f(0.1, _ver_offset+(top_the_tick_index-i)*_the_ticks);
        glEnd();
    }

    glBegin(GL_POLYGON);
    glVertex2f(0,_phi_ticks_r-0.1);
    glVertex2f(-0.02,_phi_ticks_r-0.15);
    glVertex2f(0.02,_phi_ticks_r-0.15);
    glEnd();

    glRotatef(-_phi*180/M_PI,0,0,1);

    // Altimeter

    int top_z_tick_index = floor(_z/_z_ticks_incr+(double)_n_z_ticks/2);
    double top_z_tick_pos = (top_z_tick_index-_z/_z_ticks_incr)*_z_ticks;

    glBegin(GL_POLYGON);
    glVertex2f(_phi_ticks_r+0.13, 0.02);
    glVertex2f(_phi_ticks_r+0.09, 0.0f);
    glVertex2f(_phi_ticks_r+0.13,  -0.02);
    glEnd();

    glBegin(GL_LINES);
    glVertex2f(_phi_ticks_r+0.09, _phi_ticks_r);
    glVertex2f(_phi_ticks_r+0.09, -_phi_ticks_r);
    glEnd();

    for (int i = 0; i < _n_z_ticks; i++){
        glBegin(GL_LINES);
        if ((top_z_tick_index-i)%(int)(1/_z_ticks_incr) == 0){
            glVertex2f(_phi_ticks_r+0.05, top_z_tick_pos-i*_z_ticks);
            glVertex2f(_phi_ticks_r+0.09,  top_z_tick_pos-i*_z_ticks);
        } else {
            glVertex2f(_phi_ticks_r+0.07, top_z_tick_pos-i*_z_ticks);
            glVertex2f(_phi_ticks_r+0.09,  top_z_tick_pos-i*_z_ticks);
        }
        glEnd();
    }




    // Velocity indicator

    int top_vel_tick_index = floor(_vel/_vel_ticks_incr+(double)_n_vel_ticks/2);
    double top_vel_tick_pos = (top_vel_tick_index-_vel/_vel_ticks_incr)*_vel_ticks;

    glBegin(GL_POLYGON);
    glVertex2f(-_phi_ticks_r-0.13, 0.02);
    glVertex2f(-_phi_ticks_r-0.09, 0.0f);
    glVertex2f(-_phi_ticks_r-0.13,  -0.02);
    glEnd();

    glBegin(GL_LINES);
    glVertex2f(-_phi_ticks_r-0.09, _phi_ticks_r);
    glVertex2f(-_phi_ticks_r-0.09, -_phi_ticks_r);
    glEnd();

    for (int i = 0; i < _n_z_ticks; i++){
        glBegin(GL_LINES);
        if ((top_vel_tick_index-i)%(int)(1/_vel_ticks_incr) == 0){
            glVertex2f(-_phi_ticks_r-0.05, top_vel_tick_pos-i*_vel_ticks);
            glVertex2f(-_phi_ticks_r-0.09,  top_vel_tick_pos-i*_vel_ticks);
        } else {
            glVertex2f(-_phi_ticks_r-0.07, top_vel_tick_pos-i*_vel_ticks);
            glVertex2f(-_phi_ticks_r-0.09,  top_vel_tick_pos-i*_vel_ticks);
        }
        glEnd();
    }

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, width, 0.0, height);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();


    glRotatef(_phi*180/M_PI,0,0,1);

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();


    // Heading indicator

    int top_psi_tick_index = floor(_psi/_psi_ticks_incr+(double)_n_psi_ticks/2);
    double top_psi_tick_pos = (top_psi_tick_index-_psi/_psi_ticks_incr)*_psi_ticks;

    glBegin(GL_POLYGON);
    glVertex2f(0.02,-0.8+0.13);
    glVertex2f(0.0f,-0.8+0.09);
    glVertex2f(-0.02,-0.8+0.13);
    glEnd();

    glBegin(GL_LINES);
    glVertex2f( _phi_ticks_r*0.5, -0.8+0.09);
    glVertex2f( -_phi_ticks_r*0.5,-0.8+0.09);
    glEnd();

    for (int i = 0; i < _n_psi_ticks; i++){
        glBegin(GL_LINES);
        if ((top_psi_tick_index-i)%(int)(1/_z_ticks_incr) == 0){
            glVertex2f(top_psi_tick_pos-i*_psi_ticks,-0.8+0.05);
            glVertex2f( top_psi_tick_pos-i*_psi_ticks,-0.8+0.09);
                    glEnd();

        } else {
            glVertex2f(top_psi_tick_pos-i*_psi_ticks,-0.8+0.07);
            glVertex2f(top_psi_tick_pos-i*_psi_ticks,-0.8+0.09);
                    glEnd();
        }

    }
    draw_text();

}

void gl_widget::draw_text(){
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, width, 0.0, height);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    void * font = GLUT_BITMAP_8_BY_13;

    // Altimeter text
    char alt_buffer[50];
    const char * alt_l = "ALT";
    int n = sprintf(alt_buffer, "%.1f",_z);
    glRasterPos2i(width*(0.5+_phi_ticks_r/2)+28, height/2+7);
    for (int i = 0; i < 3; i++)
    {
        glutBitmapCharacter(font, alt_l[i]);
    }

    glRasterPos2i(width*(0.5+_phi_ticks_r/2)+28, height/2-7);
    for (int i = 0; i < n; i++)
    {
        glutBitmapCharacter(font, alt_buffer[i]);
    }

    // Velocity indicator text
    char vel_buffer[50];
    const char * vel_l = "VEL";
    n = sprintf(vel_buffer, "%.1f",_vel);
    glRasterPos2i(width*(0.5-_phi_ticks_r/2)-8*6, height/2+7);
    for (int i = 0; i < 3; i++)
    {
        glutBitmapCharacter(font, vel_l[i]);
    }

    glRasterPos2i(width*(0.5-_phi_ticks_r/2)-8*(n+3), height/2-7);
    for (int i = 0; i < n; i++)
    {
        glutBitmapCharacter(font, vel_buffer[i]);
    }

    // Heading indicator text
    char psi_buffer[50];
    double psi_tmp;
    int top_psi_tick_index = floor(_psi/_psi_ticks_incr+(double)_n_psi_ticks/2);
    double top_psi_tick_pos = (top_psi_tick_index-_psi/_psi_ticks_incr)*_psi_ticks;



    for (int i = 0; i < _n_psi_ticks; i++){
        if ((top_psi_tick_index-i)%(int)(1/_z_ticks_incr) == 0){


            psi_tmp = (top_psi_tick_index-i)*_psi_ticks_incr;

            if (psi_tmp < 0)
                psi_tmp = psi_tmp+360;


            switch((int)psi_tmp){
            case 0:
                n = sprintf(psi_buffer, "N");
                break;
            case 90:
                n = sprintf(psi_buffer, "E");
                break;
            case 180:
                n = sprintf(psi_buffer, "S");
                break;
            case 270:
                n = sprintf(psi_buffer, "W");
                break;
            default:
                n = sprintf(psi_buffer, "%.0f",psi_tmp);
                break;

            }

            glRasterPos2i(width*(0.5+(top_psi_tick_pos-i*_psi_ticks)/2)-4*n, 10);

            for (int i = 0; i < n; i++)
            {
                glutBitmapCharacter(font, psi_buffer[i]);
            }

        }

    }





    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();


}



//glColor3f(0,1,0);
//glBegin(GL_POLYGON);
//glVertex2f(100,0);
//glVertex2f(100,100);
//glVertex2f(0,100);
//glEnd();

