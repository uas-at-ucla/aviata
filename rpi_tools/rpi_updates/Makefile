CC = g++
CFLAGS = -Wall -g

rpi_updates: driver.o
	$(CC) $(CFLAGS) -o rpi_updates driver.o -lssh

execute.o: driver.cpp
	$(CC) $(CFLAGS) -c driver.cpp

clean: 
	rm -rf execute rpi_updates driver.o