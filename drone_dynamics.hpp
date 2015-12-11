#ifndef _DRONE_DYNAMICS_HPP_
#define _DRONE_DYNAMICS_HPP_
#include <unistd.h>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include "console_color.h"
#include "joystick.h"


#include <QObject>
#include <QDebug>

#include "controller_client.h"
#include "param_definitions.h"



class drone_dynamics : public QObject{
    Q_OBJECT
   public:
    drone_dynamics(drone_parm* drone);
    ~drone_dynamics();

    void calc_aux();
    void calc_aux(double** parm);
    void calc_aux(int id, double** parm);

    void set_parm(double** parm);
    void set_parm(int id, double parm);

    void set_input(double * u_scaled);
    void set_input(int id, double u_scaled);

    void reset();

    void set_init_parm();
    void set_init_parm(double** init_parm);
    void set_init_parm(int id, double init_parm);

    void set_init_input();
    void set_init_input(double* init_u_scaled);
    void set_init_input(int id, double init_u_scaled);


    void init_policy_parm(policy_parm ** policy_parm);

    void input_scale();
    void input_scale(int id, double u_scaled, double * u_true);

    void calc_f(double* f, double* xd, double* x, double* u);
    void calc_g(double* g, double* xd, double* x, double* u);
    void rk4_k_calc(double *kf_n, double* kg_n, double* kf_p, double* kg_p,
                    double h);
    void euler_step();
    void euler_step(double h);
//    void rk4_step();
    void rk4_step(double h);
    void set_timestep(double h);
    
    double get_scale(int id);
    double get_state(int id);
    int get_parm_number();

    double get_timestep();
    double * get_parm();

    void report();

    void rk4_step();

    void set_controller_cmac(int u, int id, cmac_net net);
    void set_controller_parm(int u, int id, policy_parm parm);
public slots:
    void get_parm(double * parm);
    void get_state(double * state, int * state_id, int n_states);
    void step();
    void set_controller_setting(int id, int val);
    void get_controller_setting(int id, int *val);
    void get_controller_setting(int * val);
    void reset_sim();
    void init_controller_parm(int * n_controllers);
signals:
    void get_joystick_input(double * js);
    void get_policy_cmac(int u, int id, cmac_net * net);
    void get_controller_cmac_weights(int u, int id, double * weights);


   private:
    void get_control_input();
    // Pointers to drone constants
    drone_parm* p;
    //double* b;
    //double* d;
    //double* Ixx;
    //double* Iyy;
    //double* Izz;
    //double* Irotor;
    //double* m;
    //double* l;

    //double* a1_phi, *a1_the, *a1_psi, *a2_phi, *a2_the, *a3_phi, *a3_the,
        //*a3_psi;

    int _n_out;  // number of outputs
    int _n_out_tot;
    int _n_aux_out;  // number of auxillary outputs
    int _n_in;       // number of inputs

    double** _parm;
    int _n_parms;

    double* _x;
    double* _xd;
    double* _xdd;
    double* _aux;

    double* _u_true;
    double* _u_scaled;

    double* _init_parm;

    double* _init_u_true;
    double* _init_u_scaled;

    double _h;  // default timestep

    double * _l_limit; // state lower limit
    double * _u_limit; // state upper limit
    double * _spread; // state value spread

//    QTimer * _controller_parm_timer;

    int *_controller_setting;
    int *_n_controllers;
    policy_parm ** _policy_parm;

    controller_client *** _controller;

//    int ** _policy_config;
private slots:
};

#endif
