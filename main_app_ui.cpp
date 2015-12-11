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

    plot_refresh(ui->plot_state_t);
    plot_refresh(ui->plot_state_stated);


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

//    this->ui->U1_b->setValue((int)(50+js.joystickPosition(1).y*50));
//    this->ui->U2_b->setValue((int)(50+js.joystickPosition(0).x*50));
//    this->ui->U3_b->setValue((int)(50+js.joystickPosition(0).y*50));
//    this->ui->U4_b->setValue((int)(50+js.joystickPosition(1).x*50));

    emit get_drone_parm(_drone_parm_cache);
    emit get_controller_setting(_control_setting_cache,_control_name_cache);

    double zd_init_state[3] = {0,0,0};
    double zd_goal_state[1] = {0};
    int steps = 0;

    emit get_controller_status(0,0,zd_init_state,zd_goal_state,&steps);

    this->ui->init_zd_val->setText(QString::number(zd_init_state[0],'f',1));
    this->ui->init_zdd_val->setText(QString::number(zd_init_state[1],'f',1));
    this->ui->init_aux0_val->setText(QString::number(zd_init_state[2],'f',1));
    this->ui->target_zd_val->setText(QString::number(zd_goal_state[0],'f',1));
    this->ui->steps_zd_val->setText(QString::number(steps));

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


void main_app_ui::get_joystick_axis(double * axis){
    axis[0] = js.joystickPosition(1).y;
    axis[1] = js.joystickPosition(0).x;
    axis[2] = -js.joystickPosition(0).y;
    axis[3] = js.joystickPosition(1).x;
}

void main_app_ui::loop_25_hz(){
}

void main_app_ui::plot_refresh(QCustomPlot *plot){
    QVector<double> x(101), y(101); // initialize with entries 0..100
  for (int i=0; i<101; ++i)
  {
    x[i] = i/50.0 - 1; // x goes from -1 to 1
    y[i] = x[i]*x[i];  // let's plot a quadratic function
  }
  // create graph and assign data to it:
  plot->addGraph();
  plot->addGraph();
  plot->addGraph();
  plot->addGraph();
  plot->graph(0)->setData(x, y);
  // give the axes some labels:
  plot->xAxis->setLabel("x");
  plot->yAxis->setLabel("y");
  // set axes ranges, so we see all data:
  plot->xAxis->setRange(-1, 1);
  plot->yAxis->setRange(0, 1);
  plot->replot();
}

void main_app_ui::draw_log(learning_log log){

  ui->plot_state_t->graph(0)->setPen(QPen(Qt::red));
  ui->plot_state_t->graph(1)->setPen(QPen(Qt::black));
  ui->plot_state_t->graph(2)->setPen(QPen(Qt::gray));
  ui->plot_state_t->graph(3)->setPen(QPen(Qt::darkGray));

    ui->plot_state_t->graph(0)->setData(log.get_timestamp_future(),log.get_x_future());
    ui->plot_state_t->graph(1)->setData(log.get_timestamp_log(),log.get_x_log());
    ui->plot_state_t->graph(2)->setData(log.get_timestamp_log(),log.get_x_target_log());
    ui->plot_state_t->xAxis->setRange(log.get_min_t(),log.get_max_t());
    ui->plot_state_t->yAxis->setRange(-10,10);
    ui->plot_state_t->xAxis->setLabel("t[s]");
    ui->plot_state_t->yAxis->setLabel("zd[m/s]");
    QVector<double> target_level_x;
    QVector<double> target_level_y;
    target_level_x.append(0);
    target_level_x.append(log.get_max_t());
    target_level_y.append(log.get_x_target_log().last());
    target_level_y.append(log.get_x_target_log().last());
    ui->plot_state_t->graph(3)->setData(target_level_x,target_level_y);
    ui->plot_state_t->replot();

  ui->plot_state_stated->graph(0)->setPen(QPen(Qt::red));
    ui->plot_state_stated->graph(0)->setData(log.get_timestamp_future(),log.get_xd_future());
    ui->plot_state_stated->xAxis->setRange(-5,5);
    ui->plot_state_stated->yAxis->setRange(-10,10);
    ui->plot_state_stated->xAxis->setLabel("t[s]");
    ui->plot_state_stated->yAxis->setLabel("zdd[m/s2]");
    ui->plot_state_stated->replot();
}
