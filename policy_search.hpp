#ifndef _POLICY_SEARCH_HPP_
#define _POLICY_SEARCH_HPP_
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include "cmac_net.h"
#include "drone_dynamics.hpp"
#include "message_handler.hpp"
#include <string>
#include <QString>
#include <QObject>

struct policy_parm {
    // State parameters
    int id_input;           // input id
    int n_action_levels;    // number of discrete action levels
    double* action_levels;  // action levels applied to input
    QString name; // controller name for output
    int type; // 0: off, 1: deriv, 2: state, 3: state error
    int id; // controller id number

    int* id_state;  // monitored states
    int n_state;    // number of states
    int* id_goal;   // goal states
    int n_goal;     // number of goals


    // Learning parameters
    double gamma;        // discount-rate parameter
    double lambda;       // trace-decay parameter
    int max_steps;  // maximum number of steps
    double goal_thres;   // fraction threshold for goal
    double epsilon;     // probability of random action

    // CMAC parameters
    int memory_size;      // memory size to store the cmac
    int tile_resolution;  // number of segments per tile
    int n_tilings;        // number of overlapping tilings
    double alpha;         // step-size parameter


    int n_cmac_parms;

    void set_n_goal(int n);
    void set_n_state(int n);
    policy_parm();
    ~policy_parm();

    policy_parm& operator=(const policy_parm& source);
};

class policy: public QObject {
    Q_OBJECT
   public:
    policy();
    ~policy();

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
    cmac_net * get_cmac();

signals:
    void get_drone_state(double * state, int * state_id, int n_states);

   private:
    drone_dynamics* m;
    cmac_net* n;
    policy_parm* p;

    // cache variables
    double *_q; // stores local Q-values
    double* _curr_goal;
    int _action;
    double * _cmac_input;
};
#endif
