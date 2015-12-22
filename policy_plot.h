#ifndef PLOT_POLICY_H
#define PLOT_POLICY_H
#include <math.h>
#include <QApplication>
#include <QObject>
#include <QDebug>
//#include <qwtplot3d/qwt3d_types.h>
#include <qwtplot3d/qwt3d_surfaceplot.h>
#include <qwtplot3d/qwt3d_function.h>
class plotting_linker: public QObject{
    Q_OBJECT
public:
    plotting_linker();
    void request_action_val(double * control_input, double x, double xd);
signals:
    void get_action_val(double * control_input, double x, double xd);
};


class rosenbrock: public Qwt3D::Function/*, public QObject */
{
//    Q_OBJECT

public:
//    class qt_linker: public QObject{
//        Q_OBJECT
//    signals:
//        void get_action_val(double *control_input, double x, double xd);
//    } qt;

    rosenbrock(Qwt3D::SurfacePlot& pw);
    double operator()(double x, double y);
//signals:
//    void get_action_val(double * control_input, double x, double xd);
    plotting_linker linker;
private:
    double * _control_input;
};

class policy_plot : public Qwt3D::SurfacePlot{
    Q_OBJECT
public:
    policy_plot(QWidget * parent);
    rosenbrock * _rosenbrock;
public slots:
    void repaint();
    void refresh_plot();

private:
    
};
#endif










//#define POLICY_DISPLAY_ON

#ifdef POLICY_DISPLAY_ON
#ifndef POLICY_DISPLAY_H
#define POLICY_DISPLAY_H

//#include <complex>
#include <QWidget>
#include <QtOpenGL>
#include <QApplication>
#include <QMainWindow>
//#include <mgl2/mgl.h>
#include <QThread>
#include <QtSvg/QSvgGenerator>
#include "plstream.h"
#include "plplot_qt.h"

class plplot_policy_display : public QWidget
{

    Q_OBJECT
protected:
    PLINT strm;
    QtExtWidget * plot;
public:
    plplot_policy_display(QWidget * parent);
    void plot_something();
private:
    plstream *pls;


};

#endif // POLICY_DISPLAY_H
#endif //POLICY_DISPLAY_ON
