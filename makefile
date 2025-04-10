STD :=c++0x
CC := gcc
OPTIMIZATION LEVEL := 2
CXXFLAGS := -Wall -Wextra -pedantic-errors -std=$(STD) -O$(OPTIMIZATION-LEVEL) -pthread -D_GLIBCXX_USE_NANOSLEEP -fno-strict-aliasing
EXAMPLEFLAGS := -std=$(STD) -pthread -D_GLIBCXX_USE_NANOSLEEP -fno-strict-aliasing

all: note main done

note:
	@echo
	@echo Not necessarily a box2d clone
	@echo
	@echo
	@echo DOES IT WORK? DOES IT WORK? DOES IT WORK?
	@echo

main:
	$(CC) $(CXXFLAGS) ./src/main.c -o ./bin/file

done:
	@echo
	@echo
	@echo Success; Done - fertig (ziemlich)
	@echo
	@echo see ./bin directory for output executable!
	@echo
