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

    ui.show();

//    _sim_timestep_ms = 1000/_sim_frequency;

    _drone_dynamics = new drone_dynamics(&_drone_parm);
    _drone_dynamics->set_timestep((double)(_sim_timestep_f));

    _sim_timer = new QTimer(this);

    connect_signals();



    _sim_timer->start(_sim_timestep_ms_i);
}

main_app::~main_app(){
    delete[] _controller_setting;

}

void main_app::connect_signals(){
    connect(_sim_timer,SIGNAL(timeout()),_drone_dynamics,SLOT(step()));
    connect(_drone_dynamics,SIGNAL(get_joystick_input(double*)),&ui,SLOT(get_joystick_axis(double*)));

    // Interface signal routing
    connect(&ui,SIGNAL(get_drone_parm(double*)),_drone_dynamics,SLOT(get_parm(double*)));
    connect(&ui,SIGNAL(get_controller_setting(int*,QString*)),this,SLOT(get_controller_setting(int*,QString*)));
    // Controller button routing
    connect(&ui,SIGNAL(button_press_event(int)),this,SLOT(button_press_event_router(int)));

    connect(this,SIGNAL(reset_sim()),_drone_dynamics,SLOT(reset_sim()));
    connect(this,SIGNAL(set_drone_controller_setting(int,int)),_drone_dynamics,SLOT(set_controller_setting(int,int)));

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
    _policy = new policy*[4];
//    _policy_config = new int*[4];
    _n_controllers = new int[4]();

    _n_controllers[0] = 1;
    _n_controllers[1] = 0;
    _n_controllers[2] = 0;
    _n_controllers[3] = 0;

    for (int i = 0; i < 4; i++){
        _policy_parm[i] = new policy_parm[_n_controllers[i]];
        _policy[i] = new policy[_n_controllers[i]];
        for (int j = 0; j < _n_controllers[i];j++)
            _policy[i][j].set_model(&_drone_parm);
     }

    // Altitude derivative controller
    _policy_parm[0][0].name = "DERIV";
    _policy_parm[0][0].type = 1;
    _policy_parm[0][0].set_n_goal(1);
    _policy_parm[0][0].set_n_goal(1);
    _policy_parm[0][0].set_n_state(3);
    _policy_parm[0][0].id_goal[0] = 8;

    _policy_parm[0][0].id_state[0] = 8;
    _policy_parm[0][0].id_state[1] = 14;
    _policy_parm[0][0].id_state[2] = 18;

    _policy_parm[0][0].id_input = 0;
    _policy_parm[0][0].max_steps = 10000;
//    alt_rate_control.report();
//    _policy[0][0].set_model(&_drone_parm);
    _policy[0][0].set_parm(&_policy_parm[0][0]);


//    for (int i = 0; i < 4; i++){
//        _policy_parm[i] = new policy_parm[3];
//        _policy_parm[i][0].name = "OFF";
//        _policy_parm[i][1].name = "DERIV";
//        _policy_parm[i][2].name = "STATE";
//        _policy_parm[i][3].name = "ST_ERR";
//    }

}

//void main_app::get_cmac_cache(){};
//void main_app::update_cmac_cache(int id, double * weights){};



