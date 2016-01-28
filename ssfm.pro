TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    utils/iotools.cpp \
    utils/sfmtools.cpp \
    utils/cvtools.cpp \
    utils/stringtools.cpp \
    sfm/cloud.cpp \
    sfm/sfm.cpp \
    sfm/cloudpoint.cpp \
    math/quaternion.cpp \
    sba/imgproj.c \
    sba/readparams.c \
    sba/sba_chkjac.c \
    sba/sba_crsm.c \
    sba/sba_lapack.c \
    sba/sba_levmar_wrap.c \
    sba/sba_levmar.c

HEADERS += \
    utils/sfmtools.hpp \
    utils/stringtools.hpp \
    utils/iotools.hpp \
    utils/cvtools.hpp \
    sfm/cloud.hpp \
    sfm/sfm.hpp \
    sfm/cloudpoint.hpp \
    math/quaternion.hpp \
    sba/readparams.h \
    sba/imgproj.h \
    sba/compiler.h \
    sba/sba_chkjac.h \
    sba/sba.h


LIBS += -L/usr/local/lib/ -lopencv_contrib -lopencv_calib3d -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_nonfree -lopencv_features2d
#SBA使用了一个矩阵库LAPACK,Mac下用Accelerate,Win和Linux需要改成LAPACK
#http://www.netlib.org/lapack/
LIBS += -framework Accelerate
INCLUDEPATH += /usr/local/include
INCLUDEPATH+=$$PWD/utils
INCLUDEPATH+=$$PWD/sfm
INCLUDEPATH+=$$PWD/sba
INCLUDEPATH+=$$PWD/math
