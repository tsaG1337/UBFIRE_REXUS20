greaterThan(QT_MAJOR_VERSION, 4) {
    QT      += widgets serialport
    QT      += multimedia

}

TARGET = firetamer
TEMPLATE = app

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    settingsdialog.cpp \
    rexuscontrol.cpp

HEADERS += \
    mainwindow.h \
    settingsdialog.h \
    rexuscontrol.h

FORMS += \
    mainwindow.ui \
    settingsdialog.ui \
    rexuscontrol.ui

RESOURCES += \
    FireTamer.qrc

DISTFILES += \
    ../../../Gauges/Software/GaugeControl/libs/libFSUIPC_User.a

