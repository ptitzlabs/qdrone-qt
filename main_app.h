#ifndef MAIN_APP_H
#define MAIN_APP_H

#include<QObject>
#include <QVector>

#include "main_app_ui.h"
#include "policy_search.hpp"
#include "logging.h"


class main_app: public QObject{
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
    void init_drone_controller_parm(int * n_controllers);
    void cache_update_dispatch_call();
    void policy_plot_check_linker(double *z, double x, double y);
//    void get_cmac_cache();

private:

    void cycle_control_settings(int id);

    main_app_ui ui;

    QThread *** _learning_thread;
    QThread * sim_thread;
    QThread ui_thread;

    QTimer * _sim_timer;
    QTimer * _cache_timer;

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
    policy *** _policy;
    double** _cmac_weights;

    int *_controller_setting;

    int ** _memory_size_cache;
    cmac_net_parm ** _cmac_net_cache;
    double *** _cmac_weights_cache;
    
    

    void init_policy_parm();
    void init_drone_sim();
    void connect_signals();
    void spawn_threads();
    
    void init_logging();
    
    logging * log;

    

private slots:
//    void update_log();
    void button_press_event_router(int id);
    void get_controller_setting(int * setting, QString* name);
    void get_controller_status(int u, int id, double * init_state, double * goal_state, int * episode);
    void get_policy_cmac(int u, int id, cmac_net * net);
    void get_policy_cmac_weights(int u, int id, double * weights);

    void cmac_weights_cache_update(int u, int id, double * weights);
    void cache_update();


    void policy_plot_redraw(double* z, double x, double xd);
//    void update_cmac_cache(int id, double * weights);


};

#endif // MAIN_APP_H
