STD :=c++0x
CC := g++
OPTIMIZATION LEVEL := 2
CXXFLAGS := -Wall -Wextra -pedantic-errors -std=$(STD) -O$(OPTIMIZATION-LEVEL) -pthread -D_GLIBCXX_USE_NANOSLEEP -fno-strict-aliasing
EXAMPLEFLAGS := -std=$(STD) -pthread -D_GLIBCXX_USE_NANOSLEEP -fno-strict-aliasing

all: note main done

note: 
	@echo 
	@echo FUN PROJECT
	@echo 
	@echo
	@echo DUDE HAVE SOME FUN MAN!!!
	@echo

main:
	$(CC) $(CXXFLAGS) ./src/main.cpp -o ./bin/test

done:
	@echo
	@echo
	@echo Done Building Everything Successfully!
	@echo 
	@echo See ./bin directory for output executable!
	@echo
