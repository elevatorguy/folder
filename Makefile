STD :=c++0x
CC := g++
OPTIMIZATION LEVEL := 2
CXXFLAGS := -Wall -Wextra -pedantic-errors -std=$(STD) -O$(OPTIMIZATION-LEVEL) -pthread -D_GLIBCXX_USE_NANOSLEEP -fno-strict-aliasing
EXAMPLEFLAGS := -std=$(STD) -pthread -D_GLIBCXX_USE_NANOSLEEP -fno-strict-aliasing

all: note main done

note: 
	@echo 
	@echo PROJECT NAME
	@echo 
	@echo
	@echo DOES IT WORK? DOES IT WORK? DOES IT WORK?
	@echo

main:
	$(CC) $(CXXFLAGS) -o ./bin/file -I include

done:
	@echo
	@echo
	@echo Done Building Everything Successfully!
	@echo 
	@echo See ./bin directory for output executable!
	@echo
