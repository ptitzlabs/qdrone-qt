#include "main_app_ui.h"
#include "ui_main_app_ui.h"

main_app_ui::main_app_ui(QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::main_app_ui),
    _n_buttons(12){

    ui->setupUi(this);
    _buttons_tmp = new int[_n_buttons]();

    _timer_25_hz = new QTimer(this);
    _timer_50_hz = new QTimer(this);
    _timer_500_hz = new QTimer(this);

    _drone_parm_cache = new double[19]();
    _control_setting_cache = new int[4]();
    _control_name_cache = new QString[4]();

    connect(_timer_25_hz,SIGNAL(timeout()),this,SLOT(loop_25_hz()));
    connect(_timer_50_hz,SIGNAL(timeout()),this,SLOT(loop_50_hz()));
    connect(_timer_500_hz,SIGNAL(timeout()),this,SLOT(loop_500_hz()));

    _timer_25_hz->start(40);
    _timer_50_hz->start(20);
    _timer_500_hz->start(2);


}

main_app_ui::~main_app_ui(){
    delete[] _buttons_tmp;
    delete[] _drone_parm_cache;
    delete[] _control_name_cache;
    delete[] _control_setting_cache;

}

void main_app_ui::loop_500_hz(){
    for (int i = 0; i < _n_buttons; i++){
        switch(js.buttonPressed(i)){
        case 0:
            if (_buttons_tmp[i] == 1){
                _buttons_tmp[i] = 0;
                emit button_release_event(i);
//            qDebug()<<"button"<<i<<"released"<<_buttons_tmp[0]<<" "<<_buttons_tmp[1]<<" "<<_buttons_tmp[2]<<" "<<_buttons_tmp[3];
            }

            break;
        case 1:
//                std::cout<<"button "<<i<<" pressed\n";
            if (_buttons_tmp[i] == 0){
                _buttons_tmp[i] = 1;
                emit button_press_event(i);
//            qDebug()<<"button"<<i<<"pressed"<<_buttons_tmp[0]<<" "<<_buttons_tmp[1]<<" "<<_buttons_tmp[2]<<" "<<_buttons_tmp[3];
            }
            break;
        }
    }
}

void main_app_ui::loop_50_hz(){
//    std::cout<<"loop 25hz\n";
    // Updating joystick slider
    this->ui->U1_s->setValue((int)(js.joystickPosition(1).y*100));
    this->ui->U2_s->setValue((int)(js.joystickPosition(0).x*100));
    this->ui->U3_s->setValue((int)(js.joystickPosition(0).y*100));
    this->ui->U4_s->setValue((int)(js.joystickPosition(1).x*100));
}

void main_app_ui::loop_25_hz(){}

void main_app_ui::get_joystick_axis(double * axis){
    axis[0] = js.joystickPosition(1).y;
    axis[1] = js.joystickPosition(0).x;
    axis[2] = -js.joystickPosition(0).y;
    axis[3] = js.joystickPosition(1).x;

    emit get_drone_parm(_drone_parm_cache);
    emit get_controller_setting(_control_setting_cache,_control_name_cache);

    this->ui->x_l->setText(QString::number(_drone_parm_cache[0],'f',1));
    this->ui->y_l->setText(QString::number(_drone_parm_cache[1],'f',1));
    this->ui->z_l->setText(QString::number(_drone_parm_cache[2],'f',1));
    this->ui->phi_l->setText(QString::number(_drone_parm_cache[3]*180/M_PI,'f',1));
    this->ui->the_l->setText(QString::number(_drone_parm_cache[4]*180/M_PI,'f',1));
    this->ui->psi_l->setText(QString::number(_drone_parm_cache[5]*180/M_PI,'f',1));
    this->ui->xd_l->setText(QString::number(_drone_parm_cache[6],'f',3));
    this->ui->yd_l->setText(QString::number(_drone_parm_cache[7],'f',3));
    this->ui->zd_l->setText(QString::number(_drone_parm_cache[8],'f',3));
    this->ui->phid_l->setText(QString::number(_drone_parm_cache[9],'f',3));
    this->ui->thed_l->setText(QString::number(_drone_parm_cache[10],'f',3));
    this->ui->psid_l->setText(QString::number(_drone_parm_cache[11],'f',3));
    this->ui->xdd_l->setText(QString::number(_drone_parm_cache[12],'f',3));
    this->ui->ydd_l->setText(QString::number(_drone_parm_cache[13],'f',3));
    this->ui->zdd_l->setText(QString::number(_drone_parm_cache[14],'f',3));
    this->ui->phidd_l->setText(QString::number(_drone_parm_cache[15],'f',3));
    this->ui->thedd_l->setText(QString::number(_drone_parm_cache[16],'f',3));
    this->ui->psidd_l->setText(QString::number(_drone_parm_cache[17],'f',3));


    this->ui->render_widget->refresh_instruments(_drone_parm_cache[3],
            _drone_parm_cache[4],
            _drone_parm_cache[2],
            _drone_parm_cache[8],
            _drone_parm_cache[6],
            _drone_parm_cache[7],
            _drone_parm_cache[5]*180/M_PI,
            _control_name_cache);

    this->ui->render_widget->repaint();


}
