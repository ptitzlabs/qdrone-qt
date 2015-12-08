#include "dynamics_id_thread.h"

dynamics_id_thread::dynamics_id_thread(QObject * parent)
    :QThread(parent)
{
    std::cout<<"dynamics ID thread started"<<std::endl;

}

dynamics_id_thread::~dynamics_id_thread(){

}

void dynamics_id_thread::run(){}

void dynamics_id_thread::fetch_drone_parm(drone_parm * parm){
    *parm = _parm;
}



