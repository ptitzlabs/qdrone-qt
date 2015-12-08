#ifndef CONTROLLER_THREAD_H
#define CONTROLLER_THREAD_H

#include <QtCore>
#include <QThread>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "drone_dynamics.hpp"
#include "policy_search.hpp"

class controller_thread:public QThread
{
    Q_OBJECT
public:
    controller_thread(QObject * parent = 0);
    ~controller_thread();
    void train();
    void set_policy_parm(policy_parm * parm);
    void get_policy_parm(int id, policy_parm * parm);
    void get_cmac_net(cmac_net * net);

signals:
    void request_state(double * parm);
    void request_drone_parm(drone_parm * parm);
    void get_drone_state(double * state, int * state_id,int n_states);
    void get_joystick_axis_status(double * input);
    void update_controller_status(double* input, int id);
protected:
    void run() Q_DECL_OVERRIDE;
private:
    QMutex mutex;
    QWaitCondition condition;
    bool restart;
    bool abort;
    drone_parm _drone_parm;
    policy _policy;

    void set_init_condition();
private slots:
};

#endif // CONTROLLER_THREAD_H
