#include "logging.h"

learning_log::learning_log()
    :_max_log_memory(250),
      _max_future_memory(1500),
      _h_log(0.02),
      _h_future(0.02){
    _min_t = -_h_log*_max_log_memory;
    _max_t = _h_future*_max_future_memory;
}

learning_log::~learning_log(){}

void learning_log::update_log(double x, double xd, double x_target){
    _x_log.append(x);
    _xd_log.append(xd);
//    qDebug()<<xd;
//    _xd_log.append(4);
    _x_target_log.append(_goal_tmp);
    while(_x_log.size() > _max_log_memory){
        _x_log.removeFirst();
        _xd_log.removeFirst();
        _x_target_log.removeFirst();
    }
    
    if (_timestamp_log.size()<_max_log_memory)
        _timestamp_log.prepend(-_h_log*_timestamp_log.size());
}

void learning_log::update_future(QVector<double> x_future, QVector<double> xd_future, QVector<double> timestamp_future){
    if (!timestamp_future.isEmpty()){
    _x_future = x_future;
    _xd_future = xd_future;
    _timestamp_future = timestamp_future;
    
    _max_t = 5;
    
//    while(_max_t<timestamp_future.last() && _max_t <= 40)
//        _max_t*=2;
    }
    
}

void learning_log::set_x_id(int id) { _x_id = id;} 
void learning_log::set_xd_id(int id) { _xd_id = id;} 
int learning_log::get_x_id(){return _x_id;}
int learning_log::get_xd_id(){return _xd_id;}

QVector<double> learning_log::get_x_log(){return _x_log;}
QVector<double> learning_log::get_xd_log(){return _xd_log;}
QVector<double> learning_log::get_x_target_log(){return _x_target_log;}
QVector<double> learning_log::get_timestamp_log(){return _timestamp_log;}
QVector<double> learning_log::get_x_future(){return _x_future;}
QVector<double> learning_log::get_xd_future(){return _xd_future;}
QVector<double> learning_log::get_timestamp_future(){return _timestamp_future;}

double learning_log::get_min_t(){return _min_t;}
double learning_log::get_max_t(){return _max_t;}

void learning_log::set_goal_tmp(double goal){
    _goal_tmp = goal;
}


logging::logging()
{
    _z_deriv.set_x_id(8);
    _z_deriv.set_xd_id(14);
}

void logging::update_log(){
    double * state_tmp = new double[2];
    int * state_id_tmp = new int[2];

    state_id_tmp[0] = _z_deriv.get_x_id();
//    state_id_tmp[1] = _z_deriv.get_xd_id();
    state_id_tmp[1] = 14;

    state_id_tmp[1] = 14;

    double * goal_tmp = new double[1];

    emit get_goal(goal_tmp);

    emit get_drone_state(state_tmp,state_id_tmp,2);

//    state_tmp[0] = 1;
//    state_tmp[1] = 2;
    _z_deriv.update_log(state_tmp[0],state_tmp[1],goal_tmp[0]);
    emit draw_log(_z_deriv);
}

void logging::update_future(QVector<double> x_future, QVector<double> xd_future, QVector<double> timestamp_future){
    _z_deriv.update_future(x_future,xd_future,timestamp_future);
//    qDebug()<<"received!";
}

void logging::update_goal(double goal){
    _z_deriv.set_goal_tmp(goal);
}
