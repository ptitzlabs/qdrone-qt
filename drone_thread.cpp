#include "drone_thread.h"

drone_thread::drone_thread(QObject *parent)
    :QThread(parent),
      drone(0),
      h(0.1)
{
    restart = false; // controls the flow of the run() function
    abort = false;

    drone = new drone_dynamics(&parm);
}

drone_thread::~drone_thread(){
    mutex.lock(); // protects access to the abort and condition variables
    abort=true; // tell run() to stop running as soon as possible
    condition.wakeOne(); // wake up the thread if it's sleeping
    mutex.unlock();

    wait(); // wait until run() has exited
    delete drone;
}

// sim() is called by the widget whenever it needs to
void drone_thread::sim(){
    QMutexLocker locker(&mutex);
    if(!isRunning()){
        start(LowPriority);

    } else{
        restart = true;
        condition.wakeOne();
    }
}

void drone_thread::run(){
    double kk = 0;
    update_policy_parm();
    update_policy_cmac();
//    std::cout<<"unz "<<kk<<"\n";
    forever { // function body is an infinite loop
        // Start by storing the parameters in local variables. Access is protected using mutex.
        // Storing everything locally alows us to minimize the amount of code protected by mutex.
        mutex.lock();
        mutex.unlock();
//        drone->set_input(0,1);
//        drone->set_input(1,1);
//        drone->set_input(2,1);
//        drone->set_input(3,1);
        drone->rk4_step();
//        std::cout<<"unz "<<kk<<"\n";

        double parm[drone->get_parm_number()];
        for (int i = 0; i < drone->get_parm_number(); i++)
            parm[i] = drone->get_state(i);

        emit sim_status(parm);
        QThread::msleep((int)(drone->get_timestep()*1000));
        kk += 0.1;
    }
}




void drone_thread::fetch_joystick_output(double U1, double U2, double U3, double U4){
    double * input_tmp = new double[4]();
    double * goal_tmp = new double[1]();
    goal_tmp[0] = U1*10;
    input_tmp[0] = U1;
    input_tmp[1] = U2;
    input_tmp[2] = U3;
    input_tmp[3] = U4;



    alt_rate_control.get_control_input(input_tmp,drone->get_parm(),goal_tmp);

    drone->set_input(0,input_tmp[0]);
    drone->set_input(1,input_tmp[1]);
    drone->set_input(2,input_tmp[2]);
    drone->set_input(3,input_tmp[3]);

    delete[] input_tmp;
    delete[] goal_tmp;
}

void drone_thread::get_state(double * parm){
    for (int i = 0; i < drone->get_parm_number(); i++)
        parm[i] = drone->get_state(i);
}

double drone_thread::get_state(int id){
    return drone->get_state(id);
}

void drone_thread::get_state(double * state, int * state_id, int n_states){

        mutex.lock();
            for (int i = 0; i < n_states; i++)
        state[i] = drone->get_state(state_id[i]);
        mutex.unlock();
}
void drone_thread::update_policy_parm(){
    emit get_policy_parm(0,alt_rate_control.get_policy_parm());
}
void drone_thread::update_policy_cmac(){
    emit get_policy_cmac(0,alt_rate_control.get_cmac_net());
}
