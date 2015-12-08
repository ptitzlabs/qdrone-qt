#ifndef MAIN_APP_H
#define MAIN_APP_H

#include <QMainWindow>
#include <QtCore>
#include <QtConcurrent/QtConcurrent>
#include <cstdio>
#include <iostream>

#include "drone_thread.h"
#include "controller_thread.h"
#include "dynamics_id_thread.h"
#include "controller_server_thread.h"

namespace Ui {
class main_app;
}

class main_app : public QMainWindow
{
    Q_OBJECT

public:
    explicit main_app(QWidget *parent = 0);
    ~main_app();
signals:
    void fetch_joystick_output(double U1, double U2, double U3, double U4);
    void refresh_instruments(double phi, double the);
    void get_drone_parm(drone_parm * curr_parm);

private:
    Ui::main_app *ui;

    void refresh_data();

    // Drone simulation
    drone_thread sim;

    // Drone dynamics identification
    dynamics_id_thread dynamics;

    // Communication between policy search and drone dynamics
//    controller_server_thread controller_server;

    // Controller parameters
    policy_parm alt_rate_parm;

    // Controller policy search thread
    controller_thread alt_rate_control;


    // UI variables

    double x_l;
    double y_l;
    double z_l;
    double phi_l;
    double the_l;
    double psi_l;

    double xd_l;
    double yd_l;
    double zd_l;
    double phid_l;
    double thed_l;
    double psid_l;

    double xdd_l;
    double ydd_l;
    double zdd_l;
    double phidd_l;
    double thedd_l;
    double psidd_l;

    // Joystick parameters

    cJoystick js;

    void init_controller_parm();

private slots:
void update_drone_status(double * parm);
void update_controller_status(double * parm, int id);
void get_joystick_axis_status(double * axis);

void get_policy_parm(int id, policy_parm parm);
void get_policy_cmac(int id, cmac_net cmac);


};

#endif // MAIN_APP_H
