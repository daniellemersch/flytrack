# flycam makefile
# equivalent to:
# g++ -o flytrack rectedit.cc flytrack.cpp exception.cpp utils.cpp -I /usr/local/include/ -L /usr/local/lib -lopencv_highgui -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_video -lopencv_ml -lopencv_objdetect -lopencv_legacy -lopencv_calib3d -lncurses


CC = g++
OUTPUTNAME = ~/executables/Flytrack_group${D}
COMMON = ~/git/source_code/common
INCLUDE = -I ${COMMON} -I /../../include/ -I /usr/include/flycapture/
LIBS =  -L../../lib -lflycapture${D} -L /usr/local/lib -L /usr/local/share/OpenCV/3rdparty/lib -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lippicv -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core -lncurses -lX11 -lpthread -lfreeimage

CFLAGS = -O3 -Wall
#CFLAGS = -O3 -Wall

#OUTDIR = ../../bin

OBJS = ${COMMON}/netutils.o flytrack.o rectedit.o ${COMMON}/utils.o ${COMMON}/maccompat.o ${COMMON}/pthread_event.o exception.o ${COMMON}/stdafx.o ${COMMON}/cvbmpwrite.o flycam.o

${OUTPUTNAME}: ${OBJS}
	@echo [*] Linking...
	@${CC} -g -o ${OUTPUTNAME} ${OBJS} ${LIBS} ${COMMON_LIBS} 
#ifneq (${D}, d)
#	strip --strip-unneeded ${OUTPUTNAME}
#endif
#	mv ${OUTPUTNAME} ${OUTDIR}

%.o: %.cpp
	@echo [*] Compiling $<
	@${CC} -o $@ ${CFLAGS} ${INCLUDE} -c $*.cpp

%.o: %.cc
	@echo [*] Compiling $<
	@${CC} -o $@ ${CFLAGS} ${INCLUDE} -c $*.cc
	
clean_obj:
	rm -f ${OBJS}
	@echo "all cleaned up!"

clean:
#	rm -f ${OUTDIR}/${OUTPUTNAME} ${OBJS}	@echo "all cleaned up!"
	rm -f ${OUTPUTNAME} ${OBJS}
	@echo "all cleaned up!"
