CC = g++
CFLAGS = -g -Wall -Wreorder
SRCS = main.cpp
PROG = output

OPENCV = `pkg-config opencv --cflags --libs `
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)
