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
	${OBJECTDIR}/DualCameraAligner.o \
	${OBJECTDIR}/DualCameraMerger.o \
	${OBJECTDIR}/FastUndistort.o \
	${OBJECTDIR}/FoVChecker.o \
	${OBJECTDIR}/PointsCollectorChess.o \
	${OBJECTDIR}/PointsCollectorCircles.o \
	${OBJECTDIR}/SingleCalibration.o \
	${OBJECTDIR}/TwoCamerasCalibrator.o \
	${OBJECTDIR}/Undistort.o \
	${OBJECTDIR}/main.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=-pthread
CXXFLAGS=-pthread

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-L/usr/local/opencv_3.2/lib /usr/local/opencv_3.2/lib/libopencv_aruco.so /usr/local/opencv_3.2/lib/libopencv_bgsegm.so /usr/local/opencv_3.2/lib/libopencv_bioinspired.so /usr/local/opencv_3.2/lib/libopencv_calib3d.so /usr/local/opencv_3.2/lib/libopencv_ccalib.so /usr/local/opencv_3.2/lib/libopencv_core.so /usr/local/opencv_3.2/lib/libopencv_datasets.so /usr/local/opencv_3.2/lib/libopencv_dnn.so /usr/local/opencv_3.2/lib/libopencv_dpm.so /usr/local/opencv_3.2/lib/libopencv_face.so /usr/local/opencv_3.2/lib/libopencv_features2d.so /usr/local/opencv_3.2/lib/libopencv_flann.so /usr/local/opencv_3.2/lib/libopencv_fuzzy.so /usr/local/opencv_3.2/lib/libopencv_highgui.so /usr/local/opencv_3.2/lib/libopencv_imgcodecs.so /usr/local/opencv_3.2/lib/libopencv_imgproc.so /usr/local/opencv_3.2/lib/libopencv_line_descriptor.so /usr/local/opencv_3.2/lib/libopencv_ml.so /usr/local/opencv_3.2/lib/libopencv_objdetect.so /usr/local/opencv_3.2/lib/libopencv_optflow.so /usr/local/opencv_3.2/lib/libopencv_phase_unwrapping.so /usr/local/opencv_3.2/lib/libopencv_photo.so /usr/local/opencv_3.2/lib/libopencv_plot.so /usr/local/opencv_3.2/lib/libopencv_reg.so /usr/local/opencv_3.2/lib/libopencv_rgbd.so /usr/local/opencv_3.2/lib/libopencv_saliency.so /usr/local/opencv_3.2/lib/libopencv_shape.so /usr/local/opencv_3.2/lib/libopencv_stereo.so /usr/local/opencv_3.2/lib/libopencv_stitching.so /usr/local/opencv_3.2/lib/libopencv_structured_light.so /usr/local/opencv_3.2/lib/libopencv_superres.so /usr/local/opencv_3.2/lib/libopencv_surface_matching.so /usr/local/opencv_3.2/lib/libopencv_text.so /usr/local/opencv_3.2/lib/libopencv_tracking.so /usr/local/opencv_3.2/lib/libopencv_video.so /usr/local/opencv_3.2/lib/libopencv_videoio.so /usr/local/opencv_3.2/lib/libopencv_videostab.so /usr/local/opencv_3.2/lib/libopencv_xfeatures2d.so /usr/local/opencv_3.2/lib/libopencv_ximgproc.so /usr/local/opencv_3.2/lib/libopencv_xobjdetect.so /usr/local/opencv_3.2/lib/libopencv_xphoto.so

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_aruco.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_bgsegm.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_bioinspired.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_calib3d.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_ccalib.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_core.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_datasets.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_dnn.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_dpm.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_face.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_features2d.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_flann.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_fuzzy.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_highgui.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_imgcodecs.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_imgproc.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_line_descriptor.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_ml.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_objdetect.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_optflow.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_phase_unwrapping.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_photo.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_plot.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_reg.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_rgbd.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_saliency.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_shape.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_stereo.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_stitching.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_structured_light.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_superres.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_surface_matching.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_text.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_tracking.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_video.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_videoio.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_videostab.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_xfeatures2d.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_ximgproc.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_xobjdetect.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: /usr/local/opencv_3.2/lib/libopencv_xphoto.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/DualCameraAligner.o: DualCameraAligner.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv_3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/DualCameraAligner.o DualCameraAligner.cpp

${OBJECTDIR}/DualCameraMerger.o: DualCameraMerger.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv_3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/DualCameraMerger.o DualCameraMerger.cpp

${OBJECTDIR}/FastUndistort.o: FastUndistort.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv_3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/FastUndistort.o FastUndistort.cpp

${OBJECTDIR}/FoVChecker.o: FoVChecker.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv_3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/FoVChecker.o FoVChecker.cpp

${OBJECTDIR}/PointsCollectorChess.o: PointsCollectorChess.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv_3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/PointsCollectorChess.o PointsCollectorChess.cpp

${OBJECTDIR}/PointsCollectorCircles.o: PointsCollectorCircles.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv_3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/PointsCollectorCircles.o PointsCollectorCircles.cpp

${OBJECTDIR}/SingleCalibration.o: SingleCalibration.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv_3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/SingleCalibration.o SingleCalibration.cpp

${OBJECTDIR}/TwoCamerasCalibrator.o: TwoCamerasCalibrator.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv_3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/TwoCamerasCalibrator.o TwoCamerasCalibrator.cpp

${OBJECTDIR}/Undistort.o: Undistort.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv_3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Undistort.o Undistort.cpp

${OBJECTDIR}/main.o: main.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/opencv_3.2/include -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} -r ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_stitching.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_surface_matching.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_rgbd.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_dnn.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_ximgproc.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_dpm.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_bioinspired.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_xphoto.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_imgproc.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_structured_light.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_ml.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_video.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_bgsegm.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_phase_unwrapping.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_photo.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_core.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_imgcodecs.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_highgui.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_line_descriptor.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_reg.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_xobjdetect.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_xfeatures2d.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_calib3d.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_saliency.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_face.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_aruco.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_shape.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_text.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_optflow.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_plot.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_datasets.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_features2d.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_flann.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_fuzzy.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_superres.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_tracking.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_ccalib.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_videostab.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_objdetect.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_stereo.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_videoio.so
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cameramerging

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
