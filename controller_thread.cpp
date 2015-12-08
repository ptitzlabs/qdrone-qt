#include "controller_thread.h"

controller_thread::controller_thread(QObject * parent)
    :QThread(parent)
{
    restart = false;
    abort = false;
    std::cout<<"controller thread started"<<std::endl;
    emit request_drone_parm(&_drone_parm);
    _policy.set_model(&_drone_parm);
}

controller_thread::~controller_thread(){}
void controller_thread::run(){
    double input[4];
    double controller_status[4];
    double init_state[_policy.get_policy_parm()->n_state];
    int n_state = _policy.get_policy_parm()->n_state;
    int id_state[n_state];
    for (int i = 0; i < n_state; i++){
        id_state[i] = _policy.get_policy_parm()->id_state[i];
    }
    int id_input = _policy.get_policy_parm()->id_input;
    int n_goal = _policy.get_policy_parm()->n_goal;
    int id_goal[n_goal];
    for (int i = 0; i < n_goal; i++){
        id_goal[i] = _policy.get_policy_parm()->id_goal[i];
    }

    double goal[n_goal];

    //double init_state[_policy_parm.n_state];
    //std::cout<<"n_state: "<<_policy_parm->n_state<<std::endl;
    forever{
        emit get_drone_state(init_state,id_state,n_state);


        emit get_joystick_axis_status(input);
        goal[0] = input[0] * 10;
        _policy.set_init(init_state);
        _policy.set_goal(goal);


        controller_status[0] = init_state[0];
        controller_status[1] = init_state[1];
        controller_status[2] = init_state[2];
        controller_status[3] = input[0]*10;



        emit update_controller_status(controller_status,0);
        _policy.run_episode();

    }

}

void controller_thread::set_policy_parm(policy_parm * parm){
    _policy.set_parm(parm);
    //    _policy.get_policy_parm(&policy_parm);
    std::cout<<YELLOW<<_policy.get_policy_parm()->n_action_levels<<RESET<<std::endl;

}
void controller_thread::set_init_condition(){

}

void controller_thread::get_policy_parm(int id, policy_parm * parm){
    *parm =*_policy.get_policy_parm();
}
void controller_thread::get_cmac_net(cmac_net *net){
    *net = *_policy.get_cmac();
}

