#ifndef DRONE_THREAD
#define DRONE_THREAD

#include <QtCore>
#include <QThread>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "drone_dynamics.hpp"
#include "policy_search.hpp"
#include "controller_client.h"

class drone_thread:public QThread{
    Q_OBJECT

public:
    drone_thread(QObject * parent = 0); // constructor
    ~drone_thread(); // destructor

    void sim(); //

signals:
    void sim_status(double * parm); // emitted whenever the thread is done
    // Getting the controller parameters and CMAC storage
    void get_policy_parm(int id, policy_parm parm);
    void get_policy_cmac(int id, cmac_net net);

protected:
    void run() Q_DECL_OVERRIDE; // automatically called when the thread is started

private:
    QMutex mutex; // protects other data member
    QWaitCondition condition; //
    drone_parm parm;
    drone_dynamics * drone;

    void get_policy_parm_all();
    void get_cmac_net_all();
    void update_policy_parm();
    void update_policy_cmac();

    controller_client alt_rate_control;

//    policy alt_control;
//    policy pitch_control;
//    policy roll_control;
//    policy yaw_control;



    bool restart;
    bool abort;

    double h;
    
    cmac_net *_net_alt_hold;
    policy_parm * _policy_alt_hold;

private slots:
    void fetch_joystick_output(double U1, double U2, double U3, double U4);
    void get_state(double * parm);
    double get_state(int id);
    void get_state(double * state, int * state_id, int n_states);

};

#endif // DRONE_THREAD

