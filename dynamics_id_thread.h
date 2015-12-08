#ifndef DYNAMICS_ID_THREAD_H
#define DYNAMICS_ID_THREAD_H


#include <QtCore>
#include <QThread>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "drone_dynamics.hpp"

class dynamics_id_thread:public QThread{
    Q_OBJECT

public:
    dynamics_id_thread(QObject * parent = 0);
    ~dynamics_id_thread();
protected:
    void run() Q_DECL_OVERRIDE;
private:
    QMutex mutex;
    QWaitCondition condition;

    drone_parm _parm;
    bool restart;
    bool abort;
signals:
    void get_joystick_axis_status(double axis[]);
    void get_drone_state(double state[], int state_id[], int n_states);
private slots:
    void fetch_drone_parm(drone_parm * curr_parm);
};

#endif // DYNAMICS_ID_THREAD_H
