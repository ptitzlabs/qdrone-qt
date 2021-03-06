#include "drone_dynamics.hpp"

using namespace std;

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

    lower_limit = new double[n_states];
    upper_limit = new double[n_states];
    spread = new double[n_states];

    for (int i = 0; i < n_states; i++){
        lower_limit[i] = 0;
        upper_limit[i] = 0;
        spread[i] = 0;
    }

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
    for (int i = 0; i < n_states; i++){
        lower_limit[i] = parm.lower_limit[i];
        upper_limit[i] = parm.upper_limit[i];
        spread[i] = parm.spread[i];
    }

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

drone_dynamics::drone_dynamics(drone_parm * drone)
    : _n_out(6), _n_aux_out(1), _n_in(4), _h(0.1) {

    std::cout<<YELLOW<<"INITIALIZING DRONE MODEL "<<RESET<<std::endl;
    // Map pointers to supplied dynamics constants
    p = drone;
    //b = &drone.b;
    //d = &drone.d;
    //Ixx = &drone.Ixx;
    //Iyy = &drone.Iyy;
    //Izz = &drone.Izz;
    //Irotor = &drone.Irotor;
    //m = &drone.m;
    //l = &drone.l;

    //std::cout << "Drone parameters: " << std::endl;
    //std::cout << "b: " << *b << " d: " << *d << " Ixx: " << *Ixx
              //<< " Iyy: " << *Iyy << " Izz: " << *Izz << " Irotor: " << *Irotor
              //<< " m: " << *m << " l: " << *l << std::endl;

    //a1_phi = &drone.a1_phi;
    //a1_the = &drone.a1_the;
    //a1_psi = &drone.a1_psi;

    //a2_phi = &drone.a2_phi;
    //a2_the = &drone.a2_the;

    //a3_phi = &drone.a3_phi;
    //a3_the = &drone.a3_the;
    //a3_psi = &drone.a3_psi;

    // Initialize variables to store the inputs and outputs
    _n_out_tot = _n_out * 3;
    _n_parms = _n_out_tot + _n_aux_out;  // total number of output parameters
    _parm = new double *[_n_parms];      // vector storing all output
                                         // parameters
    _x = new double[_n_out];             // states
    _xd = new double[_n_out];            // derivatives
    _xdd = new double[_n_out];           // second derivatives

    _aux = new double[_n_aux_out];  // auxillary outputs

    // Mapping pointers to state outputs
    for (int i = 0; i < _n_out; i++) {
        _parm[i] = &_x[i];
        _parm[i + _n_out] = &_xd[i];
        _parm[i + _n_out * 2] = &_xdd[i];
    }

    // Mapping pointers to auxillary states
    for (int i = 0; i < _n_aux_out; i++) {
        _parm[_n_out_tot + i] = &_aux[i];
    }

    // Initialize control inputs
    _u_true = new double[_n_in];
    _u_scaled = new double[_n_in];

    // Initialize default values
    _init_parm = new double[_n_parms];
    _init_u_true = new double[_n_in];
    _init_u_scaled = new double[_n_in];

    for (int i = 0; i < _n_parms; i++) {
        *_parm[i] = 0;
    }
    for (int i = 0; i < _n_in; i++) {
        _u_scaled[i] = 0;
    }

    input_scale();
    calc_aux();
    set_init_parm();
    set_init_input();

    // Initialize state characteristic variables
    _l_limit = new double[_n_parms];
    _u_limit = new double[_n_parms];
    _spread = new double[_n_parms];

    // Set the values of characteristic variables
    _l_limit[0] = 0;  // x
    _u_limit[0] = 0;
    _l_limit[1] = 0;  // y
    _u_limit[1] = 0;
    _l_limit[2] = 0;  // z
    _u_limit[2] = 0;
    _l_limit[3] = 0;  // phi
    _u_limit[3] = 0;
    _l_limit[4] = 0;  // the
    _u_limit[4] = 0;
    _l_limit[5] = 0;  // psi
    _u_limit[5] = 0;

    _l_limit[6] = 0;  // dx
    _u_limit[6] = 0;
    _l_limit[7] = 0;  // dy
    _u_limit[7] = 0;
    _l_limit[8] = -20;  // dz
    _u_limit[8] = 10;
    _l_limit[9] = 0;  // dphi
    _u_limit[9] = 0;
    _l_limit[10] = 0;  // dthe
    _u_limit[10] = 0;
    _l_limit[11] = 0;  // dpsi
    _u_limit[11] = 0;

    _l_limit[12] = 0;  // ddx
    _u_limit[12] = 0;
    _l_limit[13] = 0;  // ddy
    _u_limit[13] = 0;
    _l_limit[14] = -G_ACC * 1.5;  // ddz
    _u_limit[14] = G_ACC * 0.5;
    _l_limit[15] = 0;  // ddphi
    _u_limit[15] = 0;
    _l_limit[16] = 0;  // ddthe
    _u_limit[16] = 0;
    _l_limit[17] = 0;  //)ddpsi
    _u_limit[17] = 0;

    _l_limit[18] = -1;  // aux0
    _u_limit[18] = 1;

    for (int i = 0; i < _n_parms; i++) {
        _spread[i] = _u_limit[i] - _l_limit[i];
    }
    calc_f(_xdd,_xd,_x,_u_true);
    report();
}
drone_dynamics::~drone_dynamics() {
    delete[] _parm;
    delete[] _x;
    delete[] _xd;
    delete[] _xdd;
    delete[] _aux;

    delete[] _u_true;
    delete[] _u_scaled;

    delete[] _init_parm;
    delete[] _init_u_true;
    delete[] _init_u_scaled;

    delete[] _l_limit;
    delete[] _u_limit;
    delete[] _spread;
}

// Calc auxillary states
void drone_dynamics::calc_aux() { calc_aux(_parm); }
void drone_dynamics::calc_aux(double **parm) {
    for (int i = 0; i < _n_aux_out; i++) calc_aux(i, parm);
}

void drone_dynamics::calc_aux(int id, double **parm) {
    switch (id) {
        case 0:
            *parm[18] = cos(*parm[3]) * cos(*parm[4]);
            break;
    }
}

// Set parameters
void drone_dynamics::set_parm(double **parm) {
    for (int i = 0; i < _n_parms; i++) set_parm(i, *parm[i]);
}

// Set individual parameters
void drone_dynamics::set_parm(int id, double parm) { *_parm[id] = parm; }

// Set input
void drone_dynamics::set_input(double *u_scaled) {
    for (int i = 0; i < _n_in; i++) set_input(i, u_scaled[i]);
}

// Set individual input
void drone_dynamics::set_input(int id, double u_scaled) {
    _u_scaled[id] = u_scaled;
    input_scale(id, u_scaled, &_u_true[id]);
}
// Reset sim
void drone_dynamics::reset() {
    for (int i = 0; i < _n_parms; i++) {
        *_parm[i] = _init_parm[i];
    }
    for (int i = 0; i < _n_in; i++) {
        _u_scaled[i] = _init_u_scaled[i];
        //_u_true[i] = _init_u_true[i];
    }
}

// Set initial parameters to current state
void drone_dynamics::set_init_parm() { set_init_parm(_parm); }
// Specify initial parameters
void drone_dynamics::set_init_parm(double **init_parm) {
    for (int i = 0; i < _n_parms; i++) _init_parm[i] = *init_parm[i];
}
// Set the initial parameters at the specified id
void drone_dynamics::set_init_parm(int id_init_parm, double init_parm) {
    _init_parm[id_init_parm] = init_parm;
}

// Set initial inputs to current input
void drone_dynamics::set_init_input() { set_init_input(_u_scaled); }

// Specify initial inputs
void drone_dynamics::set_init_input(double *init_u_scaled) {
    for (int i = 0; i < _n_in; i++) {
        set_init_input(i, init_u_scaled[i]);
    }
}

// Set the initial inputs at the specified id
void drone_dynamics::set_init_input(int id, double init_u_scaled) {
    _init_u_scaled[id] = init_u_scaled;
    input_scale(id, _init_u_scaled[id], &_init_u_true[id]);
}
// Input scaling
void drone_dynamics::input_scale() {
    for (int i = 0; i < _n_in; i++) input_scale(i, _u_scaled[i], &_u_true[i]);
}
void drone_dynamics::input_scale(int id, double u_scaled, double *u_true) {
    switch (id) {
        case 0:
            //std::cout << YELLOW << "\nDRONE PARAMETERS: " << RESET << std::endl;
            *u_true = G_ACC * p->m * (1 + 0.5 * u_scaled);
            //std::cout << "M: " << *m << "U_SCALED: " << u_scaled
                      //<< " U_TRUE: " << *u_true;
            break;
        case 1:
            *u_true = u_scaled * 0.01;
            break;
        case 2:
            *u_true = u_scaled * 0.01;
            break;
        case 3:
            *u_true = u_scaled * 0.01;
            break;
    }
}

// Equations of motion
void drone_dynamics::calc_f(double *xdd, double *xd, double *x,
                            double *u_true) {


    // Pre-calculating sines and cosines for speed
    double cphi = cos(x[3]);
    double sphi = sin(x[3]);

    double cthe = cos(x[4]);
    double sthe = sin(x[4]);

    double cpsi = cos(x[5]);
    double spsi = sin(x[5]);

    double U1m = u_true[0] / p->m;
    //std::cout<<GREEN<<"\n\n\nCURR VALUES\n";
    //for (int i = 0; i < 6; i++){
        ////std::cout<<"x["<<i<<"]: "<<x[i]<<"\t";
        ////std::cout<<"xd["<<i<<"]: "<<xd[i]<<"\n";
        ////std::cout<<"xdd["<<i<<"]: "<<xdd[i]<<std::endl;
    //}
    //for (int i = 0; i < 4; i++){
        //std::cout<<"u_true["<<i<<"]: "<<u_true[i]<<"\n";
    //}
    //std::cout<<"*m: "<<p->m<<std::endl;
    //std::cout<<RESET;
    //usleep(1000000);

    // xyz acceleration
    xdd[0] = (cphi * sthe * cpsi + sphi * spsi) * U1m;  // xdd
    xdd[1] = (cphi * sthe * spsi + sphi * cpsi) * U1m;  // ydd
    xdd[2] = -G_ACC + cphi * cthe * U1m;                // zdd

    // angle acceleration
    xdd[3] =
        xd[4] * xd[5] * p->a1_phi + x[4] *p->a2_phi + u_true[1] * p->a3_phi;  // phidd
    xdd[4] = xd[3] * xd[5] * p->a1_the + x[3] * p->a2_phi +
             u_true[2] * p->a3_phi;                            // thedd
    xdd[5] = xd[3] * xd[4] * p->a1_psi + u_true[3] *p->a3_psi;  // psidd

//    std::cout<<RED;
    for (int i = 0; i < 3; i++){
//        std::cout<<"x["<<i<<"]: "<<x[i];
        //std::cout<<"xdd["<<i<<"]: "<<xdd[i]<<std::endl;
    }
//    std::cout<<RESET<<"\n";
    //usleep(1000000);
}

void drone_dynamics::calc_g(double *g, double *xd, double *x, double *u) {
    // Since the system is time-invariant, it is sufficient to pass the
    // derivative vector as-is
    for (int i = 0; i < _n_out; i++) {
        g[i] = xd[i];
    }
}
// Euler integration
void drone_dynamics::euler_step() { euler_step(_h); }
void drone_dynamics::euler_step(double h) {
    calc_f(_xdd, _xd, _x, &*_u_true);

    for (int i = 0; i < 6; i++) {
        _xd[i] += h * _xdd[i];
        _x[i] += h * _xd[i];
    }

    _aux[0] =
        cos(_x[3]) *
        cos(_x[4]);  // cos(theta) x cos (phi), necessary for altitude control
}

// Calculating Runge-Kutta coefficients;
void drone_dynamics::rk4_k_calc(double *kf_n, double *kg_n, double *kf_p,
                                double *kg_p, double h) {
    // Allocate memory for temp values
    double *xd_tmp = new double[_n_out];
    double *x_tmp = new double[_n_out];

    for (int i = 0; i < _n_out; i++) {
        x_tmp[i] = _x[i] + h * kg_p[i];
        xd_tmp[i] = _xd[i] + h * kf_p[i];
    }

    //std::cout<<CYAN<<"\n\n\nCURR VALUES\n";
    //for (int i = 0; i < 6; i++){
        //std::cout<<"kg_p["<<i<<"]: "<<kg_p[i]<<"\t";
        //std::cout<<"kf_p["<<i<<"]: "<<kf_p[i]<<"\n";
        ////std::cout<<"xdd["<<i<<"]: "<<xdd[i]<<std::endl;
    //}
    //for (int i = 0; i < 6; i++){
        //std::cout<<"x_tmp["<<i<<"]: "<<x_tmp[i]<<"\t";
        //std::cout<<"xd_tmp["<<i<<"]: "<<xd_tmp[i]<<"\n";
        ////std::cout<<"xdd["<<i<<"]: "<<xdd[i]<<std::endl;
    //}
    //std::cout<<"h: "<<h<<std::endl;
    //std::cout<<RESET;
    //usleep(1000000);
    calc_f(&*kf_n, xd_tmp, x_tmp, _u_true);  // second derivatives at t+h
    calc_g(&*kg_n, xd_tmp, x_tmp, _u_true);  // first derivatives at t+h
    // Clear up temp values
    delete[] xd_tmp;
    delete[] x_tmp;
    //std::cout<<MAGENTA<<"\n\n\nNEXT VALUES\n";
    //for (int i = 0; i < 6; i++){
        //std::cout<<"kg_n["<<i<<"]: "<<kg_n[i]<<"\t";
        //std::cout<<"kf_n["<<i<<"]: "<<kf_n[i]<<"\n";
        ////std::cout<<"xdd["<<i<<"]: "<<xdd[i]<<std::endl;
    //}
    //std::cout<<RESET;
}

void drone_dynamics::rk4_step() { rk4_step(_h); }
void drone_dynamics::rk4_step(double h) {
    // Pre-calc the timesteps

    double h6 = h / 6;
    double h2 = h / 2;
    double hh[] = {h2, h2, h};

    // Allocate memory for rk-coefficients
    double **kf = new double *[4];
    double **kg = new double *[4];

    kf[0] = &*_xdd;
    kg[0] = &*_xd;

    for (int i = 1; i < 4; i++) {
        kf[i] = new double[_n_out];
        kg[i] = new double[_n_out];
    }
    for (int i = 1; i < 4; i++) {
        rk4_k_calc(kf[i], kg[i], kf[i - 1], kg[i - 1], hh[i - 1]);
    }

    for (int i = 0; i < 6; i++) {
        _x[i] += h6 * (kg[0][i] + 2 * kg[1][i] + 2 * kg[2][i] +
                       kg[3][i]);  // updating state values after timestep h
        _xd[i] +=
            h6 *
            (kf[0][i] + 2 * kf[1][i] + 2 * kf[2][i] +
             kf[3][i]);  // updating first derivative values after timestep h
    }

    for (int i = 3; i < 6; i++){
        if (_x[i] < -M_PI)
            _x[i] = _x[i]+2*M_PI;
        if (_x[i] > M_PI)
            _x[i] = _x[i]-2*M_PI;
    }

    calc_f(_xdd, _xd, _x, _u_true);  // updating second derivatives
    calc_aux();                      // updating auxillary values

//    std::cout<<"norm x: "<<_x[0] << " xd: " <<_xd[0] << " xdd: " << _xdd[0] <<
//               " y: "<<_x[1] << " yd: " <<_xd[1] << " ydd: " << _xdd[1] <<
//               " z: "<<_x[2] << " zd: " <<_xd[2] << " zdd: " << _xdd[2] << "\n";

//    std::cout<<"parm x: "<<*_parm[0] << " xd: " <<*_parm[6] << " xdd: " <<* _parm[12] <<
//               " y: "<<*_parm[1] << " yd: " <<*_parm[7] << " ydd: " <<* _parm[13] <<
//               " z: "<<*_parm[2] << " zd: " <<*_parm[8] << " zdd: " << *_parm[14] << "\n";



    // cleaning up
    for (int i = 1; i < 4; i++) {
        delete[] kf[i];
        delete[] kg[i];
    }
    delete[] kf;
    delete[] kg;
#ifdef BLABLA
#endif
}

double drone_dynamics::get_scale(int id) { return _spread[id]; }
//double drone_dynamics::get_l_limit(int id) { return _l_limit[id]; }
//double drone_dynamics::get_u_limit(int id) {return _u_limit[id]; }

void drone_dynamics::report() {
    std::cout << "\n###################\n"
              << "Simulation report:\n";
    std::cout << "Current state:\nx: " << _x[0] << "\txd: " << _xd[0]
              << "\txdd: " << _xdd[0] << "\tphi: " << _x[3]
              << "\tphid: " << _xd[3] << "\tphidd: " << _xdd[3]
              << "\ny: " << _x[1] << "\tyd: " << _xd[1] << "\tydd: " << _xdd[1]
              << "\tthe: " << _x[4] << "\tthed: " << _xd[4]
              << "\tthedd: " << _xdd[4] << "\nz: " << _x[2]
              << "\tzd: " << _xd[2] << "\tzdd: " << _xdd[2]
              << "\tpsi: " << _x[5] << "\tpsid: " << _xd[5]
              << "\tpsidd: " << _xdd[5] << std::endl;
    std::cout << "aux[0]: " << _aux[0] << std::endl;
    std::cout << "Current input(scaled):"
              << "\n U1: " << _u_scaled[0] << " U2: " << _u_scaled[1]
              << " U3: " << _u_scaled[2] << " U3: " << _u_scaled[3]
              << std::endl;
    std::cout << "Current input(true):"
              << "\n U1: " << _u_true[0] << " U2: " << _u_true[1]
              << " U3: " << _u_true[2] << " U3: " << _u_true[3] << std::endl;
}

double drone_dynamics::get_state(int id) { return *_parm[id]; }

void drone_dynamics::set_timestep(double h) { _h = h; }

double drone_dynamics::get_timestep() {return _h;}

double* drone_dynamics::get_parm() {return *_parm;}
int drone_dynamics::get_parm_number() { return _n_parms;}

