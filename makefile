CFLAGS = -std=c++11 -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -g -Wall -Wconversion -O3 -fPIC
LIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching svm.h svm.cpp
CC = g++

all: svm-train svm-predict svm-scale hog_des

lib: svm.o
	if [ "$(OS)" = "Darwin" ]; then \
		SHARED_LIB_FLAG="-dynamiclib -Wl,-install_name,libsvm.so.$(SHVER)"; \
	else \
		SHARED_LIB_FLAG="-shared -Wl,-soname,libsvm.so.$(SHVER)"; \
	fi; \
	$(CC) $${SHARED_LIB_FLAG} svm.o -o libsvm.so.$(SHVER)

# svm-predict: svm-predict.c svm.o
# 	$(CXX) $(CFLAGS) svm-predict.c svm.o -o svm-predict -lm
# svm-train: svm-train.c svm.o
# 	$(CXX) $(CFLAGS) svm-train.c svm.o -o svm-train -lm
# svm-scale: svm-scale.c
# 	$(CXX) $(CFLAGS) svm-scale.c -o svm-scale

# svm_pred: svm_pred.h
# 	$(CXX) $(CFLAGS) svm_pred.h -o svm_pred

# hog_des: hog_des.cpp svm.o 
# 	$(CC) $(CFLAGS) hog_des.cpp svm.o -o hog_des -lm 

svm.o: svm.cpp svm.h
	$(CC) $(CFLAGS) -c svm.cpp
clean:
	rm -f *~ svm.o svm-train svm-predict svm-scale libsvm.so.$(SHVER)

% : %.cpp
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)
