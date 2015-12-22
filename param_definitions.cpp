#include "param_definitions.h"

drone_parm::drone_parm()
    : b(1.1487e-5),
      d(2.69e-8),
      Ixx(8.5e-3),
      Iyy(8.5e-3),
      Izz(15.8e-3),
      Irotor(6.8e-5),
      m(1.2),
      l(0.24),
      n_states(19){
    // Pre-calculating various constants used in dynamic equations
    a1_phi = (Iyy - Izz) / Ixx;
    a1_the = (Izz - Ixx) / Iyy;
    a1_psi = (Ixx - Iyy) / Izz;
    a2_phi = 0;
    a2_the = 0;

    a3_phi = l / Ixx;
    a3_the = l / Iyy;
    a3_psi = 1 / Izz;

    // allocating and assigning limits

    lower_limit = new double[n_states]();
    upper_limit = new double[n_states]();
    spread = new double[n_states]();

//    for (int i = 0; i < n_states; i++){
//        lower_limit[i] = 0;
//        upper_limit[i] = 0;
//        spread[i] = 0;
//    }

    lower_limit[8] = -20;
    upper_limit[8] = 10;
    lower_limit[14] = -G_ACC*1.5;
    upper_limit[14] = G_ACC*0.5;
    lower_limit[18] = -1;
    upper_limit[18] = 1;

    for (int i = 0; i < n_states; i++) {
        spread[i] = upper_limit[i] - lower_limit[i];
    }

}
drone_parm::~drone_parm() {
    delete[] lower_limit;
    delete[] upper_limit;
    delete[] spread;
}

drone_parm& drone_parm::operator=(const drone_parm& parm){
    b = parm.b;
    d = parm.d;
    Ixx = parm.Ixx;
    Iyy = parm.Iyy;
    Izz = parm.Izz;
    Irotor = parm.Irotor;
    m = parm.m;
    l = parm.l;
    n_states = parm.n_states;

    lower_limit = new double[n_states];
    upper_limit = new double[n_states];
    spread = new double[n_states];
//    for (int i = 0; i < n_states; i++){
//        lower_limit[i] = parm.lower_limit[i];
//        upper_limit[i] = parm.upper_limit[i];
//        spread[i] = parm.spread[i];
//    }

    memcpy(lower_limit,parm.lower_limit,n_states*sizeof(double));
    memcpy(upper_limit,parm.upper_limit,n_states*sizeof(double));
    memcpy(spread,parm.spread,n_states*sizeof(double));

    a1_phi = parm.a1_phi;
    a1_the = parm.a1_the;
    a1_psi = parm.a1_psi;
    a2_phi = parm.a2_phi;
    a2_the = parm.a2_the;
    a3_phi = parm.a3_phi;
    a3_the = parm.a3_the;
    a3_psi = parm.a3_psi;
    return* this;
}

policy_parm::policy_parm()
    :n_action_levels(5),  // number of action levels
      name("DEFAULT"),      // default name
      id_state(0),          // null-pointer
      id_goal(0),           // null-pointer
      gamma(1),             // discount-rate parameter
      lambda(0.9),          // trace-decay parameter
      max_steps(1000),     // maximum number of steps
      goal_thres(0.01),      // goal threshold
      epsilon(0.2),           // random action probability
      memory_size(5000),    // cmac memory size
      tile_resolution(8),   // sub-tilings per tile
      n_tilings(10),        // number of tilings
      alpha(0.5)           // cmac learning update parameter

{
      action_levels = new double[n_action_levels];
for (int i = 0; i < 5; i++) action_levels[i] = -1 + i * 0.5;
}
policy_parm::~policy_parm() {
//    delete[] action_levels;
//    delete[] id_state;
//    delete[] id_goal;
}

void policy_parm::set_n_goal(int n) {
    n_goal = n;
    id_goal = new int[n];
    goal_input_scale = new double[n]();

}
void policy_parm::set_n_state(int n) {
    n_state = n;
    id_state = new int[n];
}
cmac_net_parm::cmac_net_parm()
    :max_num_vars(20),
max_nonzero_traces(1000),
min_trace(0.01){

}

cmac_net_parm::~cmac_net_parm(){

}

cmac_net_parm& cmac_net_parm::operator=(const cmac_net_parm& source){
    memory_size = source.memory_size;
    num_tilings = source.num_tilings;
    num_hashings = source.num_hashings;
    num_inputs = source.num_inputs;
    alpha = source.alpha;
    gamma = source.gamma;
    lambda = source.lambda;
    tile_dimension = new double[num_inputs];
    tile_sub_dimension = new double[num_inputs];
    weights = new double[memory_size];

//    for (int i = 0; i < num_inputs; i++){
//        tile_dimension[i] = source.tile_dimension[i];
//        tile_sub_dimension[i] = source.tile_sub_dimension[i];
//    }


//    for (int i = 0; i < memory_size; i++){
//       weights[i] = source.weights[i];
//    }


    memcpy(tile_dimension,source.tile_dimension,num_inputs*sizeof(double));
    memcpy(tile_sub_dimension,source.tile_sub_dimension,num_inputs*sizeof(double));
    memcpy(weights,source.weights,memory_size*sizeof(double));
    tile_resolution = source.tile_resolution;
    max_num_vars = source.max_num_vars;
    return *this;
}
