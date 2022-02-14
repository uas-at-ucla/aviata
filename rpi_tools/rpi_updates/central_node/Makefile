CC = g++
CFLAGS = -Wall -g -std=c++17

execute_ground: execute.o
	$(CC) $(CFLAGS) -o execute_ground execute.o -lssh

execute.o: execute.cpp
	$(CC) $(CFLAGS) -c execute.cpp

clean: 
	rm -rf execute_ground execute.o