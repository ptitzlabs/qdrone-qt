#ifndef _POLICY_SEARCH_HPP_
#define _POLICY_SEARCH_HPP_
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include "cmac_net.h"
#include "drone_dynamics.hpp"
#include "param_definitions.h"
#include "message_handler.hpp"
#include <string>
#include <QString>
#include <QObject>
#include <QTimer>
#include <QMutex>
#include <QThread>
#include <QVector>
#include <QDebug>
#include "controller_client.h"

//class controller_client;


class policy: public QObject {
    Q_OBJECT
   public:
    policy();
    ~policy();

    void set_u(int u);
    void set_id(int id);

    void set_parm(policy_parm* parm);
    void set_model(drone_parm* parm);


    void set_state_parm(int id[], int n);
    void set_goal_parm(int id[], int n);

    void set_goal(double* goal);
    void set_init(double* init);

    void train();
    void run_episode();
    void run_step();

    bool with_probability(double p);

    double dist_to_goal();
    bool goal_reached();
    void calc_q();
    void calc_q(int hash);
    void calc_cmac_input();

    int calc_action();
    void report();

    void fun_test(int, int);

    policy_parm* get_policy_parm();
    void get_controller_status(double *init_stat, double *goal_stat, int *steps_stat);
    cmac_net get_cmac();
    void wtf();
    void get_action_val(double *z, double x, double xd);

public slots:

    void learn();
    void cache_update();
    void get_goal(double * goal);

signals:
    void get_drone_state(double * state, int * state_id, int n_states);
    void get_joystick_input(double * js);
    void cmac_weights_cache_update(int u, int id, double * weights);
    void update_future(QVector<double> x_future, QVector<double> xd_future, QVector<double> timestamp_future);
    void update_goal(double goal);

   private:

    void set_init();
    drone_dynamics* m;
    cmac_net* n;
    policy_parm* p;

    // cache variables
    double *_q; // stores local Q-values
    double* _curr_goal;
    double _init_dist;
    int _action;
    double * _cmac_input;
    double * _init_state;
    double * _goal_state;
    double * _full_init_state;
    int _steps;


    int _u;
    int _id;

    QVector<double> _x_log;
    QVector<double> _xd_log;
    QVector<double> _timestamp_future;
    controller_client _test_controller;

    double* _goal_tmp;
    double* _state_tmp;
    double* _input_tmp;

    QMutex _weight_mutex;
};
#endif
