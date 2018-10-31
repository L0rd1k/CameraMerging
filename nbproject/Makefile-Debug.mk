#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/FastUndistort.o \
	${OBJECTDIR}/FoVChecker.o \
	${OBJECTDIR}/PointsCollectorChess.o \
	${OBJECTDIR}/PointsCollectorCircles.o \
	${OBJECTDIR}/SingleCalibration.o \
	${OBJECTDIR}/Undistort.o \
	${OBJECTDIR}/main.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-L/usr/local/opencv3.2/lib -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_calib3d -lopencv_ccalib -lopencv_core -lopencv_datasets -lopencv_dnn -lopencv_dpm -lopencv_face -lopencv_features2d -lopencv_flann -lopencv_fuzzy -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_line_descriptor -lopencv_ml -lopencv_objdetect -lopencv_optflow -lopencv_phase_unwrapping -lopencv_photo -lopencv_plot -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_shape -lopencv_stereo -lopencv_stitching -lopencv_structured_light -lopencv_superres -lopencv_surface_matching -lopencv_text -lopencv_tracking -lopencv_video -lopencv_videoio -lopencv_videostab -lopencv_xfeatures2d -lopencv_ximgproc -lopencv_xobjdetect -lopencv_xphoto

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/FastUndistort.o: FastUndistort.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/FastUndistort.o FastUndistort.cpp

${OBJECTDIR}/FoVChecker.o: FoVChecker.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/FoVChecker.o FoVChecker.cpp

${OBJECTDIR}/PointsCollectorChess.o: PointsCollectorChess.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/PointsCollectorChess.o PointsCollectorChess.cpp

${OBJECTDIR}/PointsCollectorCircles.o: PointsCollectorCircles.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/PointsCollectorCircles.o PointsCollectorCircles.cpp

${OBJECTDIR}/SingleCalibration.o: SingleCalibration.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/SingleCalibration.o SingleCalibration.cpp

${OBJECTDIR}/Undistort.o: Undistort.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Undistort.o Undistort.cpp

${OBJECTDIR}/main.o: main.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
