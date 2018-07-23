

LINKERS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_stitching

output: main.o
	g++ main.o -o output $(LINKERS) 

main.o: main.cpp
	g++ -c main.cpp -o main.o

clean: 
	rm *.o output
