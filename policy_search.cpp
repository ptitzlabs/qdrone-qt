#include "policy_search.hpp"


policy::policy()
    : m(0),  // NULL pointer for model
      n(0),   // NULL pointer for CMAC
      p(0),  // NULL pointer for policy parameters
      _q(0),
      _cmac_input(0)
{}

policy::~policy() {
    delete m;  // clear up the model
    delete n;  // clear up the cmac
    delete[] _q;
    delete[] _cmac_input;
}

policy_parm& policy_parm::operator =(const policy_parm& source){
    id_input = source.id_input;
    n_action_levels = source.n_action_levels;
    action_levels = new double[n_action_levels];
    for (int i = 0; i < n_action_levels; i++)
        action_levels[i] = source.action_levels[i];
    name = source.name;
    id = source.id;
    n_state = source.n_state;
    id_state = new int[n_state];
    for (int i = 0; i < n_state; i++)
        id_state[i] = source.id_state[i];
    n_goal = source.n_goal;
    id_goal = new int[n_goal];
    for (int i = 0; i < n_goal; i++)
        id_goal[i] = source.id_goal[i];

    gamma = source.gamma;
    lambda = source.lambda;
    max_steps = source.max_steps;
    epsilon = source.epsilon;

    memory_size = source.memory_size;
    tile_resolution = source.tile_resolution;
    n_tilings = source.n_tilings;
    alpha = source.alpha;

//    n_cmac_params = source.n_cmac_parms;
}

void policy::set_parm(policy_parm* policy_parm) {
    msg::begin_text();
    std::cout << YELLOW << "Setting policy parameters" << RESET
              << std::endl;
    p = &*policy_parm;
    // Initializing a CMAC net to store the policy
    // CMAC tile dimensions to accomodate monitored states
    p->n_cmac_parms =
        p->n_state + p->n_goal;  // add up the monitored states and goals
    double tile_dimension[p->n_cmac_parms];  // init tile dimensions

    // for (int i = 0; i < 19; i++) {
    // std::cout << "state: " << i << " " << m->get_scale(i) << "\n";
    //}
    for (int i = 0; i < p->n_state; i++) {
        tile_dimension[i] =
            m->get_scale(p->id_state[i]);  // fill up the state dimensions
                                           // std::cout << tile_dimension[i];
    }

    for (int i = 0; i < p->n_goal; i++) {
        tile_dimension[p->n_state + i] =
            m->get_scale(p->id_goal[i]);  // fill up the goal dimensions
    }

    // Creating a new cmac instance and initializing the parameters
    n = new cmac_net;
    n->parm_init(
        p->n_cmac_parms,     // both state and goal are used as inputs
        tile_dimension,      // tile dimensions are the expected spread of
                             // values
        p->tile_resolution,  // tile resolution is the number of partitions
        p->memory_size,      // allocated memory size
        p->n_tilings,        // number of overlapping tiles
        p->n_action_levels,  // number of action levels
        p->alpha,            // update rate parameter alpha
        p->gamma,            // trace decay parameter gamma
        p->lambda);          // discount-rate parameter
    n->report();

    // Init cache variables
    _q = new double[p->n_action_levels];
    _cmac_input = new double[p->n_cmac_parms];
    msg::end_text();

    _test_controller.set_parm(*p);
    _test_controller.set_cmac_net(*n);
}
void policy::set_model(drone_parm* sim_parm) {
    msg::begin_text();
    m = new drone_dynamics(sim_parm);
    msg::end_text();
}

//void policy::fun_test(int n, int o) {
    // m->set_input(n,o);
    // m->rk4_step();
//}
void policy::set_goal(double* goal) { _curr_goal = goal; }
void policy::set_init(double* init) {
    for (int i = 0; i < p->n_state; i++) {
        m->set_init_parm(p->id_state[i], init[i]);
    }
    m->reset();
}

void policy::set_state_parm(int id[], int n) {
    p->id_state = new int[n];
    p->n_state = n;
    for (int i = 0; i<n; i++) p->id_state[i] = id[i];
}
void policy::set_goal_parm(int id[], int n) {
    p->id_goal = new int[n];
    p->n_goal = n;
    for (int i = 0; i<n; i++) p->id_goal[i] = id[i];
}

bool policy::with_probability(double p) {
    return p > ((float)rand()) / RAND_MAX;
}

double policy::dist_to_goal() {
    double goal_dist = 0;
    int id;
    double dist_tmp = 0;
    ;

    for (int i = 0; i < p->n_goal; i++) {
        id = p->id_state[i];  // get the state id value
        dist_tmp = (m->get_state(id) - _curr_goal[i]) /
                   m->get_scale(id);  // check the distance between current
                                      // state and the goal and normalize
        goal_dist +=
            dist_tmp * dist_tmp;  // add squared value to current distance
    }
    return sqrt(goal_dist);  // return normalized vector magnitude
}

bool policy::goal_reached() {
    return dist_to_goal() < p->goal_thres;  // check if current distance to goal
                                            // is less than threshold value
}

void policy::calc_q() {
    for (int i = 0; i < p->n_action_levels; i++) {
        n->return_value(&_q[i], i);
    }
}
void policy::calc_q(int hash) { n->return_value(&_q[hash], hash); }
int policy::calc_action() {
    int index = 0;
    float max_val = _q[0];
    int num_ties = 1;
    for (int i = 1; i < p->n_action_levels; i++) {
        if (_q[i] >= max_val) {
            if (_q[i] > max_val) {  // find max Q for current action
                max_val = _q[i];
                index = i;
            } else {
                num_ties++;
                if (rand() % num_ties == 0) {  // randomly break ties
                    max_val = _q[i];
                    index = i;
                }
            }
        }
    }
    if (with_probability(p->epsilon)) {  // or take a random guess
        index = rand() % p->n_action_levels;
    }
    return index;
}

void policy::calc_cmac_input() {
    // Supply current state as CMAC input
    for (int i = 0; i < p->n_state; i++) {
        _cmac_input[i] = m->get_state(p->id_state[i]);
    }
    // Supply goal state as CMAC input
    for (int i = 0; i < p->n_goal; i++) {
        _cmac_input[p->n_state + i] = _curr_goal[i];
    }
}

void policy::get_goal(double * goal){
    goal[0] = _curr_goal[0];
    qDebug()<<"curr goal: "<<goal[0];
}

void policy::run_episode() {
        _x_log.clear();
        _xd_log.clear();
        _timestamp_future.clear();
//    std::cout << YELLOW << "Running episode: " << RESET << std::endl;
    m->reset();                      // reset model
    n->clear_traces();               // clear traces
    calc_cmac_input();               // update CMAC input
    n->generate_tiles(_cmac_input);  // generate tiles
    calc_q();                        // calculate Q-values
    _action = calc_action();         // find optimal action

    int step = 0;  // initial step number
    while (!goal_reached() && step < p->max_steps) {
        run_step();
        step++;
    }

    double weight_checksum_episode=0;
    double weight_checksum_controller=0;

    double q_checksum_episode = 0;
    double q_checksum_controller = 0;

    for (int i = 0; i < n->get_memory_size(); i++)
        weight_checksum_episode+=n->get_weights()[i];

    m->reset();
    n->clear_traces();

    _test_controller.set_cmac_net_weights(n->get_weights());
    double state_tmp[p->n_state];
    double goal_tmp[p->n_goal];
    double input_tmp;
    step = 0;

    while(!goal_reached() && step < p->max_steps){
    calc_cmac_input();
    n->generate_tiles(_cmac_input);
    calc_q();
    q_checksum_episode += _q[0];
        _x_log.append(m->get_state(p->id_state[0]));
        _xd_log.append(m->get_state(p->id_state[1]));
        _timestamp_future.append(0.1*step);
        for (int i = 0; i < p->n_state;i++)
            state_tmp[i] = m->get_state(p->id_state[i]);
        for (int i = 0; i < p->n_goal; i++)
            goal_tmp[i] = _curr_goal[i];
        _test_controller.get_control_input(&input_tmp,state_tmp,goal_tmp);
        q_checksum_controller+=_test_controller.get_q(0);
        m->set_input(p->id_input,
                     input_tmp);
        m->rk4_step();
        step++;
    }
    for (int i = 0; i < _test_controller.get_cmac_net()->get_memory_size(); i++)
        weight_checksum_controller+=_test_controller.get_cmac_net()->get_weights()[i];

//    qDebug()<< weight_checksum_controller << weight_checksum_episode;
    qDebug()<< q_checksum_controller << q_checksum_episode;

    emit update_future(_x_log,_xd_log,_timestamp_future);
    emit update_goal(_goal_state[0]);

//    std::cout << BOLDWHITE << "##########\n" << step << "\n##########" << RESET
//              << std::endl;
    _steps = step;
}

void policy::run_step() {
    // std::cout << YELLOW << "Running step: " << RESET << std::endl;
    n->drop_traces();           // drop traces
    n->update_traces(_action);  // update traces for the current action
    // std::cout << "\nACTION: " << _action
    //<< " INPUT: " << p->action_levels[_action]<<std::endl;
    m->set_input(
        p->id_input,                 // specify model input
        p->action_levels[_action]);  // set model input to current action
    // m->report();
    m->rk4_step();  // execute model step

//    std::cout << RED << "Current distance to goal:" << dist_to_goal() << RESET
//              << std::endl;

    double reward =
        -dist_to_goal();  // calculate the reward, based on distance value
    double delta = reward - _q[_action];  // substract old Q(k) value
    calc_cmac_input();  // update CMAC input with the new model state
    n->generate_tiles(_cmac_input);  // generate new CMAC tiles
    calc_q();                        // calculate new Q(k+1) values
    _action = calc_action();         // calculate the action for the next step
    if (!goal_reached()) {
        delta += p->gamma * _q[_action];  // calc the change in Q(k+1)
    }
    n->quick_update(delta);  // update Q(k+1) in the CMAC
    calc_q(_action);         // calculate updated Q(k+1)
}

policy_parm* policy::get_policy_parm(){
    return p;
}

cmac_net policy::get_cmac(){
    return *n;
}

void policy::report() {
    std::cout << "\n=====================" << std::endl;
    std::cout << "CONTROLLER PARAMETERS" << std::endl;
    std::cout << "=====================" << std::endl;
    m->report();
    n->report();
}

void policy::get_controller_status(double *init_state, double *goal_state, int *steps){
    for (int i = 0; i < p->n_state; i++){
        init_state[i] = _init_state[i];
    }
    for (int i = 0; i < p->n_goal; i++){
        goal_state[i] = _goal_state[i];
    }

    *steps = _steps;

}

void policy::learn(){
    double input[4];
//    QTimer *timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(cache_update()));
//    timer->start(1000);
//    double init_state[p->n_state];
    _init_state = new double[p->n_state];
    _goal_state = new double[p->n_goal];



    forever{
        emit get_drone_state(_init_state,p->id_state,p->n_state);
        emit get_joystick_input(input);
        for (int i = 0; i < p->n_goal; i++)
            _goal_state[i] = input[p->id_input]*p->goal_input_scale[i];
        set_init(_init_state);
        set_goal(_goal_state);
        run_episode();
        cache_update();
//    qDebug()<< "Controller "<<_u<<","<<_id<<" cache update dispatch";
    }

}

void policy::set_u(int u){
    _u = u;
}

void policy::set_id(int id){
    _id = id;
}

void policy::cache_update(){
//    qDebug()<< "Controller "<<_u<<","<<_id<<" cache update dispatch received";
    emit cmac_weights_cache_update(_u,_id,n->get_cmac_net_parm().weights);
}
