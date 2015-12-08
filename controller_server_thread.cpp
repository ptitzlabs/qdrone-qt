#include "controller_server_thread.h"


controller_server_thread::controller_server_thread(QObject * parent)
    :QThread(parent),
      _portno(1234)
{
    restart = false;
    abort = false;

    _sockfd = socket(AF_INET,SOCK_STREAM,0);
    if (_sockfd < 0)
        std::cout<<"ERROR opening socket"<<std::endl;
    bzero((char*) &_serv_addr,sizeof(_serv_addr));
    _serv_addr.sin_family = AF_INET;
    _serv_addr.sin_addr.s_addr = INADDR_ANY;
    _serv_addr.sin_port = htons(_portno);
    if (bind(_sockfd,
             (struct sockaddr *) &_serv_addr,
             sizeof(_serv_addr))<0){
        std::cout<<"ERROR on binding"<<std::endl;
    }

    listen(_sockfd,5);
    _clilen = sizeof(_cli_addr);
    _newsockfd = accept(_sockfd,
                        (struct sockaddr *) &_cli_addr,
                        &_clilen);

    if(_newsockfd<0){
        std::cout<< "ERROR reading from socket" <<std::endl;
    }
}

controller_server_thread::~controller_server_thread(){
    mutex.lock();
    abort = true;
    condition.wakeOne();
    mutex.unlock();

    wait();
}

void controller_server_thread::run(){


}
void controller_server_thread::get_current_policy_parm(int id, policy_parm parm){
//    switch(id){
//    case 0:
//        emit update_current_policy_parm(id,alt_rate_control_cache);
//    }

}
void controller_server_thread::get_current_policy_cmac(int id, cmac_net cmac){
//    switch(id){
//    case 0:
//        emit update_current_policy_parm(id,alt_rate_control_cache);
//    }
}
