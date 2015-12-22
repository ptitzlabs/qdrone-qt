#include "controller_client.h"

controller_client::controller_client()
{

}

controller_client::~controller_client(){
    delete[] _cmac_input;
}

void controller_client::set_parm(policy_parm parm){
    _policy_parm = parm;
    _cmac_input = new double[_policy_parm.n_state+_policy_parm.n_goal];
    _q = new double[_policy_parm.n_action_levels];
}
void controller_client::set_cmac_net(cmac_net net){
    _cmac_net = net;
}
void controller_client::set_cmac_net_weights(double * weights){
    _cmac_net.set_weights(weights);
}

void controller_client::calc_cmac_input(double * state, double * goal){
        // Supply current state as CMAC input
    for (int i = 0; i < _policy_parm.n_state; i++) {
        _cmac_input[i] = state[i];
    }
    // Supply goal state as CMAC input
    for (int i = 0; i < _policy_parm.n_goal; i++) {
        _cmac_input[_policy_parm.n_state + i] = goal[i];
    }
}

void controller_client::calc_q() {
    for (int i = 0; i < _policy_parm.n_action_levels; i++) {
        _cmac_net.return_value(&_q[i], i);
    }
}
int controller_client::calc_action() {
    int index = 0;
    float max_val = _q[0];
    int num_ties = 1;
    for (int i = 1; i < _policy_parm.n_action_levels; i++) {
        if (_q[i] >= max_val) {
            if (_q[i] > max_val) {  // find max Q for current action
                max_val = _q[i];
                index = i;
            } else {
                num_ties++;
                if (rand() % num_ties == 0) {  // randomly break ties
                    max_val = _q[i];
                    index = i;
//                    qDebug()<<"tie";
                }
            }
        }
    }
    return index;
}

void controller_client::get_control_input(double* input, double * client_state, double * goal_state){
    calc_cmac_input(client_state, goal_state);
//    qDebug()<<_cmac_input[0]<<_cmac_input[1]<<_cmac_input[2]<<_cmac_input[3];
    _cmac_net.generate_tiles(_cmac_input);
    calc_q();
    int _action = calc_action();
    input[_policy_parm.id_input] = _policy_parm.action_levels[_action];
//    for (int i = 0; i < p.n_state;i++){
//        _cmac_input[i] = client_state[_policy_parm.id_state[i]];
//    }
//    for (int i = 0; i < _policy_parm.n_goal;i++){
//        _cmac_input[_policy_parm.n_state+i] = goal_state[i];
//    }
//    _cmac_net.generate_tiles(_cmac_input);
    
        
}

policy_parm * controller_client::get_policy_parm(){ return  &_policy_parm; }
cmac_net * controller_client::get_cmac_net(){return &_cmac_net;}
double controller_client::get_q(int n){return _q[n];}
//void controller_client::set_controller_policy_parm(){
//}

//void controller_client::set_cmac_weights(float * weights){
//    n->set_weights(weights);
//}
