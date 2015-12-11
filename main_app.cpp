#include "main_app.h"



main_app::main_app():
_sim_frequency_i(50),
  _sim_frequency_f((double)_sim_frequency_i),
_sim_timestep_ms_i((int)(1000/_sim_frequency_i)),
_sim_timestep_ms_f(1000/_sim_frequency_f),
_sim_timestep_f(1/_sim_frequency_f){


    _controller_setting = new int[4]();
    // Initializing controller policies
    init_policy_parm();


//    _sim_timestep_ms = 1000/_sim_frequency;
    init_drone_sim();

    _cache_timer = new QTimer(this);

    connect_signals();



    _sim_timer->start(_sim_timestep_ms_i);
    _cache_timer->start(100);
    _learning_thread[0][0]->start();
    ui.show();
}

main_app::~main_app(){
    delete[] _controller_setting;

}

void main_app::init_drone_sim(){

    log = new logging;

    _drone_dynamics = new drone_dynamics(&_drone_parm);
    _drone_dynamics->set_timestep((double)(_sim_timestep_f));
    _drone_dynamics->init_controller_parm(_n_controllers);
    _drone_dynamics->init_policy_parm(_policy_parm);

    for (int i = 0; i < 4; i++){
        for (int j = 0; j <_n_controllers[i];j++){
            _drone_dynamics->set_controller_cmac(i,j,_policy[i][j]->get_cmac());
            _drone_dynamics->set_controller_parm(i,j,_policy_parm[i][j]);
        }
    }


    _sim_timer = new QTimer(this);
}

void main_app::get_policy_cmac(int u, int id, cmac_net *net){
    *net = _policy[u][id]->get_cmac();
}

void main_app::get_policy_cmac_weights(int u, int id, double *weights){
    for (int i = 0; i < _memory_size_cache[u][id]; i++){
//        weights[i] = 100;
//        weights[i] = _policy[i][id]->get_cmac().get_weight(i);
        weights[i] = _cmac_weights_cache[u][id][i];
//        weights[i] = 10;
    }
//    qDebug() << "hi! " << weights[0];
}

//void main_app::init_logging(){
//    z_deriv.set_x_id(8);
//    z_deriv.set_xd_id(14);
    
//}


void main_app::connect_signals(){
    qRegisterMetaType<QVector<double> >("QVector<double>");

    connect(log,SIGNAL(get_drone_state(double*,int*,int)),_drone_dynamics,SLOT(get_state(double*,int*,int)));
    connect(_sim_timer,SIGNAL(timeout()),log,SLOT(update_log()));
    connect(log,SIGNAL(draw_log(learning_log)),&ui,SLOT(draw_log(learning_log)));



    connect(_sim_timer,SIGNAL(timeout()),_drone_dynamics,SLOT(step()));
    connect(_cache_timer,SIGNAL(timeout()), this,SLOT(cache_update()));
    connect(_drone_dynamics,SIGNAL(get_joystick_input(double*)),&ui,SLOT(get_joystick_axis(double*)));

    // Interface signal routing
    connect(&ui,SIGNAL(get_drone_parm(double*)),_drone_dynamics,SLOT(get_parm(double*)));
    connect(&ui,SIGNAL(get_controller_setting(int*,QString*)),this,SLOT(get_controller_setting(int*,QString*)));
    // Controller button routing
    connect(&ui,SIGNAL(button_press_event(int)),this,SLOT(button_press_event_router(int)));

    connect(this,SIGNAL(reset_sim()),_drone_dynamics,SLOT(reset_sim()));
    connect(this,SIGNAL(set_drone_controller_setting(int,int)),_drone_dynamics,SLOT(set_controller_setting(int,int)));

    connect(&ui,SIGNAL(get_controller_status(int,int,double*,double*,int*)),
            this,SLOT(get_controller_status(int,int,double*,double*,int*)));
    
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < _n_controllers[i]; j++){
            // Connecting policy search to drone sim and to inputs
            connect(_policy[i][j],SIGNAL(get_drone_state(double*,int*,int)),
                    _drone_dynamics,SLOT(get_state(double*,int*,int)));
            connect(_policy[i][j],SIGNAL(get_joystick_input(double*)),
                    &ui,SLOT(get_joystick_axis(double*)));
            // Connecting policy search to UI
            // Connecting policy search threads
            connect(_learning_thread[i][j],SIGNAL(started()),
                    _policy[i][j],SLOT(learn()));
            // Cache update
            connect(this,SIGNAL(cache_update_dispatch_call()),
                    _policy[i][j],SLOT(cache_update()));

            connect(_policy[i][j],SIGNAL(cmac_weights_cache_update(int,int,double*)),
                    this,SLOT(cmac_weights_cache_update(int,int,double*)));

            connect(_policy[i][j],SIGNAL(update_future(QVector<double>,QVector<double>,QVector<double>)),
                    log,SLOT(update_future(QVector<double>,QVector<double>,QVector<double>)));
            connect(log,SIGNAL(get_goal(double*)),_policy[i][j],SLOT(get_goal(double*)));
            connect(_policy[i][j],SIGNAL(update_goal(double)),log,SLOT(update_goal(double)));
        }
    }

    connect(_drone_dynamics,SIGNAL(get_controller_cmac_weights(int,int,double*)),
            this,SLOT(get_policy_cmac_weights(int,int,double*)));

//    connect(this,SIGNAL(draw_log(learning_log)),&ui,SLOT(draw_log(learning_log)));





}

void main_app::button_press_event_router(int id){
    switch (id) {

    // Simulation control
    case 2:
        // Reset sim
        emit reset_sim();
        break;

    // Controller selection
    case 4:
        // Cycle through throttle control settings
        cycle_control_settings(0);
        break;
    case 5:
        // Cycle through roll control settings
        cycle_control_settings(1);
        break;
    case 6:
        // Cycle through pitch control settings
        cycle_control_settings(2);
        break;
    case 7:
        // Cycle through yaw control settings
        cycle_control_settings(3);
        break;

    default:
        break;
    }

}

void main_app::cycle_control_settings(int id){
    _controller_setting[id]++;
    if(_controller_setting[id]>_n_controllers[id])
        _controller_setting[id] = 0;
    emit set_drone_controller_setting(id,_controller_setting[id]);
}

void main_app::get_controller_setting(int *setting, QString * name){
    for (int i = 0; i < 4; i++){
        setting[i] = _controller_setting[i];
        if (setting[i] > 0)
            name[i] = _policy_parm[i][setting[i]-1].name;
        else
            name[i] = "OFF";
    }

}

void main_app::init_policy_parm(){
    _policy_parm = new policy_parm*[4];
    _policy = new policy**[4];
    _learning_thread = new QThread**[4];
//    _policy_config = new int*[4];
    _n_controllers = new int[4]();
//    _cmac_net_cache = new cmac_net_parm * [4];

    _n_controllers[0] = 1;
    _n_controllers[1] = 0;
    _n_controllers[2] = 0;
    _n_controllers[3] = 0;

    for (int i = 0; i < 4; i++){
        _policy_parm[i] = new policy_parm[_n_controllers[i]];
//        _cmac_net_cache[i] = new cmac_net_parm[_n_controllers[i]];
        _policy[i] = new policy*[_n_controllers[i]];
        _learning_thread[i] = new QThread * [_n_controllers[i]]();
        for (int j = 0; j < _n_controllers[i];j++){
            _policy[i][j] = new policy;
            _learning_thread[i][j] = new QThread;
            _policy[i][j]->moveToThread(_learning_thread[i][j]);
            _policy[i][j]->set_u(i);
            _policy[i][j]->set_id(j);
            _policy[i][j]->set_model(&_drone_parm);
        }
     }

    // Altitude derivative controller
    _policy_parm[0][0].name = "DERIV";
    _policy_parm[0][0].type = 1;
    _policy_parm[0][0].set_n_goal(1);
    _policy_parm[0][0].set_n_goal(1);
    _policy_parm[0][0].set_n_state(3);
    _policy_parm[0][0].id_goal[0] = 8;
    _policy_parm[0][0].goal_input_scale[0] = 10; // input scaling

    _policy_parm[0][0].id_state[0] = 8;
    _policy_parm[0][0].id_state[1] = 14;
    _policy_parm[0][0].id_state[2] = 18;

    _policy_parm[0][0].id_input = 0;
    _policy_parm[0][0].max_steps = 10000;
//    alt_rate_control.report();
//    _policy[0][0].set_model(&_drone_parm);
    _policy[0][0]->set_parm(&_policy_parm[0][0]);

    for (int i = 0; i < 4; i++){
        for (int j = 0; j < _n_controllers[i]; j++){
//            _cmac_net_cache[i][j] = _policy[i][j]->get_cmac().get_cmac_net_parm();
        }
    }
//    for (int i = 0; i < 4; i++){
//        _policy_parm[i] = new policy_parm[3];
//        _policy_parm[i][0].name = "OFF";
//        _policy_parm[i][1].name = "DERIV";
//        _policy_parm[i][2].name = "STATE";
//        _policy_parm[i][3].name = "ST_ERR";
//    }

    _cmac_weights_cache = new double**[4];
    _memory_size_cache = new int*[4];
    for(int i = 0; i < 4; i++){
        _cmac_weights_cache[i] = new double*[_n_controllers[i]];
        _memory_size_cache[i] = new int[_n_controllers[i]];
        for(int j = 0; j < _n_controllers[i]; j++){
            _cmac_weights_cache[i][j] = new double[_policy[i][j]->get_cmac().get_cmac_net_parm().memory_size]();
            _memory_size_cache[i][j] = _policy[i][j]->get_cmac().get_cmac_net_parm().memory_size;
        }
    }

}

void main_app::get_controller_status(int u, int id, double *init_state, double *goal_state, int *episode){
    _policy[u][id]->get_controller_status(init_state,goal_state,episode);
}

void main_app::cmac_weights_cache_update(int u, int id, double* weight){
//    for (int i = 0; i < _policy[u][id]->get_cmac().get_cmac_net_parm().memory_size; i++){
    for (int i = 0; i < _memory_size_cache[u][id]; i++){
        _cmac_weights_cache[u][id][i] = weight[i];
//        if (weight[i] > 1)
//            qDebug()<< weight[i];
    }
//    _cmac_net_cache[u][id] = net_parm;
//    qDebug()<<"CMAC "<<u<<" "<<id <<" updated";
}

void main_app::cache_update(){
    emit cache_update_dispatch_call();
//    qDebug()<<"Update request dispatched";
}


//void main_app::get_cmac_cache(){};
//void main_app::update_cmac_cache(int id, double * weights){};



