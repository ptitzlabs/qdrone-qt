#include "policy_plot.h"

rosenbrock::rosenbrock(Qwt3D::SurfacePlot &pw):
    Function(pw){
    _control_input = new double;

}

double rosenbrock::operator ()(double x, double y){
    emit linker.request_action_val(_control_input,x,y);
//    emit get_action_val(_control_input,x,y);
//    *_control_input = sin(x)*sin(y);
    return *_control_input;
//    return log((1-x)*(1-x) + 100 * (y-x*x)*(y-x*x))/8;
}

policy_plot::policy_plot(QWidget * parent):
SurfacePlot(parent){
//    setTitle("demo");
    _rosenbrock = new rosenbrock(*this);
    _rosenbrock->setMesh(80,20);
    _rosenbrock->setDomain(-8,8,-4,4);
    _rosenbrock->setMinZ(-1);
    _rosenbrock->setMaxZ(1);
    _rosenbrock->create();

    setRotation(30,0,15);
    setScale(1,1,1);
    setShift(0.15,0,0);
    setZoom(0.9);


//    setCoordinateStyle(BOX);

    updateData();
    updateGL();
}


void policy_plot::refresh_plot(){
    for (unsigned i = 0; i!=coordinates()->axes.size(); ++i){
        coordinates()->axes[i].setMajors(7);
        coordinates()->axes[i].setMinors(4);
    }

    coordinates()->axes[0].setLabelString("zd");
    coordinates()->axes[1].setLabelString("zdd");
    updateData();
    updateGL();
}

int counterr;
void policy_plot::repaint(){
    _rosenbrock->create();
    updateData();
    updateGL();
    counterr++;
//    qDebug()<<"repaint works OK"<<counterr;

}

plotting_linker::plotting_linker(){

}

void plotting_linker::request_action_val(double *control_input, double x, double xd){
    emit get_action_val(control_input,x,xd);
}



#ifdef POLICY_DISPLAY_ON
plplot_policy_display::plplot_policy_display(QWidget * parent)
    :QWidget(parent)
{
//    plot = new QtExtWidget(QT_DEFAULT_X,QT_DEFAULT_Y,this);

    plmkstrm(&strm);
    plsdev("exqqt");
//    setCentralWidget(plot);
//    plsetqtdev(plot);
    plinit();
    pladv(0);
//    plresize(200,200);
//    setCentralWidget(plot);
    plot_something();
}

void plplot_policy_display::plot_something(){
//    plot->clearWidget();

    int NSIZE = 100;
    PLFLT x[NSIZE],y[NSIZE];
    PLFLT xmin = 0., xmax = 1., ymin = 0., ymax = 100.;
    int i;

    for (int i = 0; i < NSIZE; i++){
        x[i] = (PLFLT)(i)/(PLFLT)(NSIZE - 1);
        y[i] = ymax * x[i] * x[i];
    }
    plenv(xmin, xmax, ymin, ymax, 0,0);
    pllab("x","y=100 x#u2#d","SIMPLE PLPLOT DEMO");
    plline(NSIZE,x,y);
}


//void mgls_prepare2d(mglData *a, mglData *b=0, mglData *v=0)
//{
//    register long i,j,n=50,m=20,i0;
//    if(a) a->Create(n,m);
//    if(b) b->Create(n,m);
//    if(v) { v->Create(9); v->Fill(-1,1); }
//    float x,y;
//    for(i=0;i<n;i++) for(j=0;j<m;j++)
//    {
//        x = i/(n-1.); y = j/(m-1.); i0 = i+n*j;
//        if(a) a->a[i0] = 0.6*sin(2*M_PI*x)*sin(3*M_PI*y)+0.4*cos(3*M_PI*x*y);
//        if(b) b->a[i0] = 0.6*cos(2*M_PI*x)*cos(3*M_PI*y)+0.4*cos(3*M_PI*x*y);
//    }
//}

#endif
