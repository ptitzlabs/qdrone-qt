#ifndef CONTROLLER_SERVER_THREAD_H
#define CONTROLLER_SERVER_THREAD_H

#include <QtCore>
#include <QThread>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "policy_search.hpp"
#include "cmac_net.h"

class controller_server_thread:public QThread{
    Q_OBJECT

public:
    controller_server_thread(QObject * parent = 0);
    ~controller_server_thread();
signals:
    void update_current_policy_parm(int id, policy_parm parm);
    void update_current_policy_cmac(int id, cmac_net cmac);

protected:
    void run() Q_DECL_OVERRIDE;

private:
QMutex mutex;
QWaitCondition condition;

bool restart;
bool abort;

int _sockfd;
int _newsockfd;
int _portno;
socklen_t _clilen;

struct sockaddr_in _serv_addr;
struct sockaddr_in _cli_addr;

policy alt_rate_control_cache;
policy phi_rate_control_cache;
policy the_rate_control_cache;
policy psi_rate_control_cache;

private slots:
 void get_current_policy_parm(int id, policy_parm parm);
 void get_current_policy_cmac(int id, cmac_net cmac);


};

#endif // CONTROLLER_SERVER_THREAD_H
