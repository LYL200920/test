QT -= gui

TEMPLATE = lib
CONFIG += c++17 staticlib

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH = ../

CONFIG(release, debug|release) {
        DESTDIR = ./release/lib
        OBJECTS_DIR = ./release/obj
        MOC_DIR = ./release/moc

        DEFINES += QT_MESSAGELOGCONTEXT
}

SOURCES += \
    text.cpp \

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target
