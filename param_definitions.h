#ifndef PARAM_DEFINITIONS
#define PARAM_DEFINITIONS
#include <QString>
#include <QObject>
#define PI 3.1415926535897932384626433832795
#define G_ACC 9.80665
struct drone_parm{
    double b;
    double d;
    double Ixx;
    double Iyy;
    double Izz;
    double Irotor;
    double m;
    double l;

    double * lower_limit;
    double * upper_limit;
    double * spread;

    int n_states;
    double a1_phi, a1_the, a1_psi, a2_phi, a2_the, a3_phi, a3_the, a3_psi;


    // Constructor
    drone_parm();
    drone_parm& operator=(const drone_parm& d);
    ~drone_parm();
};

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

    double * goal_input_scale; // used to scale inputs


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
struct cmac_net_parm{
    int memory_size;
    int num_tilings;
    int num_hashings;
    int num_inputs;
    double alpha;
    double gamma;
    double lambda;
    double * tile_dimension;
    double * tile_sub_dimension;
    double * weights;
    int tile_resolution;
    int max_num_vars;

    cmac_net_parm();
    ~cmac_net_parm();

    cmac_net_parm& operator=(const cmac_net_parm& source);
};
#endif // PARAM_DEFINITIONS

