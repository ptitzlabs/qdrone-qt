#include "main_app.h"
#include "ui_main_app.h"

main_app::main_app(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::main_app)
{
    ui->setupUi(this);
//    cJoystick js;

    // Connecting policy search thread to UI
    connect(&alt_rate_control,SIGNAL(update_controller_status(double*,int)),
            this,SLOT(update_controller_status(double*,int)));
    // Connecting simulation thread to ui
    connect(&sim, SIGNAL(sim_status(double*)),
            this,SLOT(update_drone_status(double*)));

    // Connecting policy search to dynamics identification
    connect(&alt_rate_control,SIGNAL(request_drone_parm(drone_parm* )),
            &dynamics,SLOT(fetch_drone_parm(drone_parm* )));
    // Connecting policy search to simulation thread
    connect(&alt_rate_control,SIGNAL(get_drone_state(double*, int*, int)),
            &sim,SLOT(get_state(double*,int*, int)));

    // Connecting drone client to policy search thread
    connect(&sim,SIGNAL(get_policy_parm(int,policy_parm)),
            this,SLOT(get_policy_parm(int,policy_parm)));
    connect(&sim,SIGNAL(get_policy_cmac(int,cmac_net)),
            this,SLOT(get_policy_cmac(int,cmac_net)));

    // Connecting policy search thread to joystick input
    connect(&alt_rate_control,SIGNAL(get_joystick_axis_status(double*)),
            this,SLOT(get_joystick_axis_status(double*)));    
    // Donnecting simulation thread to joystick input
    connect(this,SIGNAL(fetch_joystick_output(double,double,double,double)),
            &sim, SLOT(fetch_joystick_output(double, double, double, double)));
//    connect(this,SIGNAL(refresh_instruments(double,double)),this->ui->render_widget,SLOT(refresh_instruments(double,double)));

    drone_parm drone;
    sim.start();
    dynamics.start();
    init_controller_parm();
    alt_rate_control.start();
//    controller_server.start();


    std::cout<<"still running!";
}

main_app::~main_app()
{
    delete ui;
}

//void main_app::fetch_joystick_output(double U1, double U2, double U3, double U4){
//    U1 =js.joystickPosition(1).y;
//    U2 =js.joystickPosition(0).y;
//    U3 =js.joystickPosition(0).x;
//    U4 =js.joystickPosition(1).x;
//}

void main_app::update_drone_status(double * parm){
//    std::cout<<"lookie! "<<parm[0]<<std::endl;
//    std::cout<<"parm x: "<<parm[0] << " xd: " <<parm[6] << " xdd: " <<parm[12] <<
//               " y: "<<parm[1] << " yd: " <<parm[7] << " ydd: " <<parm[13] <<
//               " z: "<<parm[2] << " zd: " <<parm[8] << " zdd: " <<parm[14] << "\n";

//    std::cout<< parm[0] << "," <<parm[1] <<std::endl;

    this->ui->x_l->setText(QString::number(parm[0],'f',1));
    this->ui->y_l->setText(QString::number(parm[1],'f',1));
    this->ui->z_l->setText(QString::number(parm[2],'f',1));
    this->ui->phi_l->setText(QString::number(parm[3]*180/M_PI,'f',1));
    this->ui->the_l->setText(QString::number(parm[4]*180/M_PI,'f',1));
    this->ui->psi_l->setText(QString::number(parm[5]*180/M_PI,'f',1));
    this->ui->xd_l->setText(QString::number(parm[6],'f',3));
    this->ui->yd_l->setText(QString::number(parm[7],'f',3));
    this->ui->zd_l->setText(QString::number(parm[8],'f',3));
    this->ui->phid_l->setText(QString::number(parm[9],'f',3));
    this->ui->thed_l->setText(QString::number(parm[10],'f',3));
    this->ui->psid_l->setText(QString::number(parm[11],'f',3));
    this->ui->xdd_l->setText(QString::number(parm[12],'f',3));
    this->ui->ydd_l->setText(QString::number(parm[13],'f',3));
    this->ui->zdd_l->setText(QString::number(parm[14],'f',3));
    this->ui->phidd_l->setText(QString::number(parm[15],'f',3));
    this->ui->thedd_l->setText(QString::number(parm[16],'f',3));
    this->ui->psidd_l->setText(QString::number(parm[17],'f',3));

    this->ui->U1_s->setValue((int)(js.joystickPosition(1).y*100));
    this->ui->U2_s->setValue((int)(js.joystickPosition(0).x*100));
    this->ui->U3_s->setValue((int)(js.joystickPosition(0).y*100));
    this->ui->U4_s->setValue((int)(js.joystickPosition(1).x*100));
    emit fetch_joystick_output(js.joystickPosition(1).y,
                               js.joystickPosition(0).x,
                               -js.joystickPosition(0).y,
                               js.joystickPosition(1).x);
    this->ui->render_widget->refresh_instruments(parm[3],
            parm[4],
            parm[2],
            parm[8],
            parm[6],
            parm[7],
            parm[5]*180/M_PI);
    this->ui->render_widget->repaint();


//    std::cout<<(int)(js.joystickPosition(0).x*100)<<" "<<(int)(js.joystickPosition(0).y*100)<<" "<<
//               (int)(js.joystickPosition(1).x*100)<<" "<<(int)(js.joystickPosition(1).y*100)<<std::endl;

}

void main_app::update_controller_status(double * parm, int id){
    this->ui->init_zd_val->setText(QString::number(parm[0],'f',3));
    this->ui->init_zdd_val->setText(QString::number(parm[1],'f',3));
    this->ui->init_aux0_val->setText(QString::number(parm[2],'f',3));
    this->ui->target_zd_val->setText(QString::number(parm[3],'f',3));

}

void main_app::refresh_data(){
    double parm[] ={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
    this->ui->x_l->setText(QString::number(parm[0],'f',3));
    this->ui->y_l->setText("y_l");
    this->ui->z_l->setText("z_l");
    this->ui->phi_l->setText("phi_l");
    this->ui->the_l->setText("the_l");
    this->ui->psi_l->setText("psi_l");
}

void main_app::init_controller_parm(){
    drone_parm * drone;
//    emit fetch_drone_parm(drone);
//    alt_rate_control.set_model(drone);
    alt_rate_parm.set_n_goal(1);
    alt_rate_parm.set_n_state(3);
    alt_rate_parm.id_goal[0] = 8;

    alt_rate_parm.id_state[0] = 8;
    alt_rate_parm.id_state[1] = 14;
    alt_rate_parm.id_state[2] = 18;

    alt_rate_parm.id_input = 0;
    alt_rate_parm.max_steps = 10000;
//    alt_rate_control.report();
    alt_rate_control.set_policy_parm(&alt_rate_parm);
}

void main_app::get_joystick_axis_status(double * axis){
    axis[0] = js.joystickPosition(1).y;
    axis[1] = js.joystickPosition(0).x;
    axis[2] = -js.joystickPosition(0).y;
    axis[3] = js.joystickPosition(1).x;
}

void main_app::get_policy_parm(int id, policy_parm parm){
    switch(id){
    case 0:
        parm = alt_rate_parm;
        break;
    }

}

void main_app::get_policy_cmac(int id, cmac_net cmac){
    switch(id){
    case 0:
        alt_rate_control.get_cmac_net(&cmac);
    }

}
