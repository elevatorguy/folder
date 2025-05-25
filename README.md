# Simple 2D Constraint Solver

This is a simple constraint solver and physics engine originally written in C++ for use with cmake.

To see it in action, check out the [demo](https://github.com/ange-yaghi/scs-2d-demo).

`make`:

```
ECHO is off.
PROJECT NAME
ECHO is off.
ECHO is off.
DOES IT WORK? DOES IT WORK? DOES IT WORK?
ECHO is off.
icpx -Wall -Wextra -pedantic-errors -std=c++0x -O -pthread -D_GLIBCXX_USE_NANOSLEEP -fno-strict-aliasing -o ./bin/file -I include
icpx: error: no input files
make: *** [main] Error 1
```

This project is presently header files; to compile successfully, `.cpp` file(s) need to be written and the makefile updated.