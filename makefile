###########################################################
#
#	Makefile for Computer Graphics Ray Tracer
#	@file main.cpp
#	@author Aidan Becker
#	@islandID ABecker2
#	@professor Dr. Scott King
#	@class COSC 4328-001 Computer Graphics
#	@version 1.0
#
###########################################################
Compiler =gcc -std=c++11 -Wall -fopenmp
Optimizations =-Ofast -march=native -funroll-loops
Profiler =-pg -fprofile-arcs
LDLIBS =-lpthread -lm -lstdc++
Remove =rm
Object =main.cpp -o
Name =main.out
Arguments =1n.scn
perfFileName =profinfo.txt

all:
	$(Compiler) $(Optimizations) $(Object) release-$(Name) $(LDLIBS) 
	$(Compiler) $(Profiler) $(Object) debug-$(Name) $(LDLIBS) 

fast:
	$(Compiler) $(Optimizations) $(Object) $(Name) $(LDLIBS)

debug:
	$(Compiler) $(Profiler) $(Object) $(Name) $(LDLIBS)
	
clean:
	$(Remove) $(Name)

run-fast:
	./release-$(Name) $(Arguments)

run-profile:
	./debug-$(Name) $(Arguments)

remake:
	$(Remove) $(Name)
	$(Compiler) $(Object) $(Name) $(LDLIBS)

perflog:
	gprof debug-$(Name) gmon.out > $(perfFileName)
