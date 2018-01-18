QT += core
QT -= gui

CONFIG += c++11

TARGET = Random_Dot_Markers
CONFIG += console
CONFIG -= app_bundle

INCLUDEPATH += /usr/local/include/opencv2
LIBS += -L/usr/local/lib \
    -lopencv_core \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_features2d \
    -lopencv_calib3d \
    -lopencv_imgcodecs \
    -lopencv_videoio \
    -lopencv_video \

TEMPLATE = app

SOURCES += main.cpp \
    detectcircles.cpp \
    dot.cpp \
    dotmarkers.cpp \
    keyframe.cpp \
    posegrabh.cpp

HEADERS += \
    dot.h \
    dotmarkers.h \
    detectcircles.h \
    posetracker.h \
    keyframe.h \
    posegraph.h
