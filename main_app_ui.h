#ifndef MAIN_APP_UI_H
#define MAIN_APP_UI_H

#include <QMainWindow>
#include <QtCore>

#include <cstdio>
#include <iostream>

#include "joystick.h"
#include "qcustomplot.h"

#include "logging.h"


namespace Ui {
class main_app_ui;
}

class main_app_ui : public QMainWindow
{
    Q_OBJECT
public:
    explicit main_app_ui(QWidget *parent = 0);
    ~main_app_ui();

    void run();

signals:
    void get_drone_parm(double *parm);

    void button_release_event(int button);
    void button_press_event(int button);
    void get_controller_setting(int * setting, QString * name);
    void get_controller_status(int u, int id, double * init_stat, double *goal_stat, int * steps_stat);

private:
    Ui::main_app_ui * ui;
    cJoystick js;

    int _n_buttons;
    int * _buttons_tmp;

    QTimer *_timer_25_hz;
    QTimer *_timer_50_hz;
    QTimer *_timer_500_hz;

    double * _drone_parm_cache;
    int * _control_setting_cache;
    QString * _control_name_cache;

    void plot_refresh(QCustomPlot * plot);

private slots:
    void get_joystick_axis(double * axis);
    void loop_25_hz();
    void loop_50_hz();
    void loop_500_hz();

    void draw_log(learning_log log);

};

#endif // MAIN_APP_UI_H
