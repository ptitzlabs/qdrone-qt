#ifndef CONTROLLER_CLIENT_H
#define CONTROLLER_CLIENT_H

#include "policy_search.hpp"

class controller_client
{
public:
    controller_client();
    ~controller_client();
    void set_parm(policy_parm parm);
    void set_cmac_net(cmac_net cmac);
    void get_control_input(double * input, double* client_state, double * goal_state);
    void calc_cmac_input(double *state, double * goal);
    void calc_q();
    int calc_action();

    policy_parm get_policy_parm();
    cmac_net get_cmac_net();

//    void set_controller_policy_parm();
//    void set_cmac_weights(float * weights);
//    void get_input(float * input, float * current_state, float * target);
private:
    policy_parm _policy_parm;
    cmac_net _cmac_net;

    double * _cmac_input;
    double * _q;
//    policy_parm _policy_parm;
//    cmac_net _cmac_net;
};

#endif // CONTROLLER_CLIENT_H
