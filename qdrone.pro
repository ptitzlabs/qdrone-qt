#-------------------------------------------------
#
# Project created by QtCreator 2015-11-24T23:36:14
#
#-------------------------------------------------

QT       += core gui widgets opengl printsupport
CONFIG += g++11
TARGET = qdrone
INCLUDEPATH += /usr/include
DEPENDPATH = $$INCLUDEPATH

LIBS += -L/usr/local/lib -lGLEW -lglfw -lfreetype -lglut -lGLU -lqwtplot3d-qt5

#greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport
TEMPLATE = app

SOURCES += main.cpp\
    main_app.cpp \
    joystick.cc \
    gl_widget.cpp \
    policy_search.cpp \
    main_app_ui.cpp \
    cmac_net.cpp \
    controller_client.cpp\
    drone_dynamics.cpp \
    param_definitions.cpp\
    message_handler.cpp \
    qcustomplot.cpp \
    logging.cpp \
    policy_plot.cpp
#        main_app.cpp \
#    drone_thread.cpp \
#    gl_widget.cpp \
#    drone_dynamics.cpp \
#    joystick.cc \
#    controller_thread.cpp \
#    dynamics_id_thread.cpp \
#    policy_search.cpp \
#    cmac_net.cpp \
#    message_handler.cpp \
#    controller_server_thread.cpp \
#    controller_client.cpp

#    qutils.cpp

HEADERS  += main_app.h \
    joystick.h \
    gl_widget.h \
    policy_search.hpp \
    main_app_ui.h \
    qutils.h \
    RL_headers.h \
    cmac_net.h \
    controller_client.h \
    drone_dynamics.hpp \
    param_definitions.h\
    message_handler.hpp \
    console_color.h \
#main_app.h \
#    drone_thread.h \
#    gl_widget.h \
#    drone_dynamics.hpp \
#    console_color.h \
#    joystick.h \
#    controller_thread.h \
#    dynamics_id_thread.h \
#    policy_search.hpp \
#    cmac_net.h \
#    message_handler.hpp \
#    controller_server_thread.h \
#    controller_client.h
    qcustomplot.h \
    logging.h \
    policy_plot.h

FORMS    += main_app_ui.ui
