#ifndef CONTROLLER_CLIENT_H
#define CONTROLLER_CLIENT_H

#include <QObject>
#include <QDebug>
//#include "policy_search.hpp"
#include "param_definitions.h"
#include "cmac_net.h"

struct policy_parm;

class controller_client: public QObject{
    Q_OBJECT
public:
    controller_client();
    ~controller_client();
    void set_parm(policy_parm parm);
    void set_cmac_net(cmac_net cmac);
    void set_cmac_net_weights(double * weights);
    void get_control_input(double * input, double* client_state, double * goal_state);
    void calc_cmac_input(double *state, double * goal);
    void calc_q();
    int calc_action();

    policy_parm * get_policy_parm();
    cmac_net * get_cmac_net();
    double get_q(int n);

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
