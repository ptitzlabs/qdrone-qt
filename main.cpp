//#include "main_app.h"
#include <QApplication>
#include <cstdio>
#include <iostream>
#include <GL/glut.h>
#include "main_app.h"


int main(int argc, char *argv[])
{
    glutInit( & argc, argv );
    QApplication a(argc, argv);
    main_app app;
//    main_app w;
//    w.show();
    a.exit();


    return a.exec();
}
