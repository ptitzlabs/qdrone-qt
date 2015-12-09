#ifndef MAIN_APP_H
#define MAIN_APP_H

#include<QObject>

#include "main_app_ui.h"
#include "policy_search.hpp"

class main_app: QObject{
    Q_OBJECT
public:
    main_app();
    ~main_app();

    void run();


signals:
    void get_drone_parm(double *parm);
    void get_joystick_axis(double * axis);
    void set_drone_controller_setting(int id, int val);
    void reset_sim();
//    void get_cmac_cache();

private:

    void cycle_control_settings(int id);

    main_app_ui ui;

    QThread ** learning_thread;
    QThread * sim_thread;
    QThread ui_thread;

    QTimer * _sim_timer;

    int _sim_frequency_i;
    double _sim_frequency_f;
    int _sim_timestep_ms_i;
    double _sim_timestep_ms_f;
    double _sim_timestep_f;

    drone_parm _drone_parm;
    drone_dynamics * _drone_dynamics;

    int * _n_controllers;
    int **_policy_config;

    policy_parm ** _policy_parm;
    policy ** _policy;
    double** _cmac_weights;

    int *_controller_setting;

    void init_policy_parm();
    void connect_signals();

private slots:
    void button_press_event_router(int id);
    void get_controller_setting(int * setting, QString* name);
//    void update_cmac_cache(int id, double * weights);


};

#endif // MAIN_APP_H
