#-------------------------------------------------
#
# Project created by QtCreator 2014-06-05T20:48:57
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HandSkeleton
TEMPLATE = app

# ------------------------------------------------------------------------------
# OpenNI2 & NiTE2 _
INCLUDEPATH += "C:\\Program Files (x86)\\OpenNI2\\Include" \
                             "C:\\Program Files (x86)\\PrimeSense\\NiTE2\\Include"
LIBS += -L"C:/Program Files (x86)/OpenNI2/Lib" -lOpenNI2
LIBS += -L"C:/Program Files (x86)/PrimeSense/NiTE2/Lib" -lNiTE2
# _

# OpenCV _
INCLUDEPATH += "C:/opencv/ocv249/opencv/build/include"
LIBS += -L"C:/opencv/ocv249/opencv/build/x86/vc11/lib" -lopencv_core249 \
                                                                                            -lopencv_highgui249 \
                                                                                            -lopencv_imgproc249
# _

# Ogre _
INCLUDEPATH += "C:/ogrevc11/include"
LIBS += -L"C:/ogrevc11/lib/Release" -lOgreMain
# _

# Boost _
INCLUDEPATH += "C:/boostvc11"
LIBS += -L"C:/boostvc11/lib32-msvc-11.0"
# ------------------------------------------------------------------------------

SOURCES += main.cpp\
    mainwindow.cpp \
    api/recognizer.cpp \
    api/hw/sensordevice.cpp \
    api/learn/handhypothesis.cpp \
    api/learn/handmodel.cpp \
    ogrewidget.cpp

HEADERS  += mainwindow.h \
    api/recognizer.h \
    api/hw/sensordevice.h \
    api/learn/handhypothesis.h \
    api/learn/handmodel.h \
    ogrewidget.h

FORMS    += mainwindow.ui
