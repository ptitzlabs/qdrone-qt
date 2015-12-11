#ifndef LOGGING_H
#define LOGGING_H

#include <QObject>
#include <QVector>
#include <QDebug>

class learning_log{
public:
    learning_log();
    ~learning_log();
    
    void update_log(double x, double xd, double x_target);
    void update_future(QVector<double> x_future, QVector<double> xd_future, QVector<double> timestamp_future);
    void set_x_id(int id);
    void set_xd_id(int id);
    int get_x_id();
    int get_xd_id();

    QVector<double> get_x_log();
    QVector<double> get_xd_log();
    QVector<double> get_x_target_log();
    QVector<double> get_timestamp_log();
    QVector<double> get_x_future();
    QVector<double> get_xd_future();
    QVector<double> get_timestamp_future();

    double get_min_t();
    double get_max_t();

    void set_goal_tmp(double goal);
private:
    int _max_log_memory;
    int _max_future_memory;
    
    double _min_x;
    double _max_x;
    double _min_xd;
    double _max_xd;
    double _min_t;
    double _max_t;
    
    int _x_id;
    int _xd_id;
   
    
    double _h_log;
    double _h_future;
    
    QVector<double> _x_log;
    QVector<double> _xd_log;
    QVector<double> _x_target_log;
    
    QVector<double> _x_future;
    QVector<double> _xd_future;
    
//    QVector<double> _episode_x_log;
//    QVector<double> _episode_xd_log;
    QVector<double> _timestamp_log;
    QVector<double> _timestamp_future;

    double _goal_tmp;
};

class logging : public QObject{
    Q_OBJECT
public:
    logging();
    void init_logging();
signals:
    void get_goal(double * goal);
    void get_drone_state(double * state, int * state_id, int n_states);
    void draw_log(learning_log log);
private:
    learning_log _z_deriv;
private slots:
//    void draw_log();
    void update_log();
//    void update_future();
    void update_future(QVector<double> x_future, QVector<double> xd_future, QVector<double> timestamp_future);
    void update_goal(double goal);
};

#endif // LOGGING_H
