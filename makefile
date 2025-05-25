STD := c17
CC := icx
OPTIMIZATION-LEVEL := 2
CXXFLAGS := -Wall -Wextra -Qstd=$(STD) -O$(OPTIMIZATION-LEVEL) -D_GLIBCXX_USE_NANOSLEEP -fno-strict-aliasing -fsanitize=address,undefined,bounds,pointer-overflow -ferror-limit=100
EXAMPLEFLAGS := -std=$(STD) -pthread -D_GLIBCXX_USE_NANOSLEEP -fno-strict-aliasing -fsanitize=address,undefined,bounds,pointer-overflow,leak

all: note main done

note:
	@echo
	@echo Not necessarily a box2d clone
	@echo
	@echo
	@echo DOES IT WORK? DOES IT WORK? DOES IT WORK?
	@echo

main:
	$(CC) $(CXXFLAGS) ./src/clutch_constraint.c \
	./src/fixed_position_constraint.c \
	./src/gravity_force_generator.c \
	./src/optimized_nsv_rigid_body_system.c \
	./src/simple_gear_constraint.c \
	./src/utilities.c \
	./src/conjugate_gradient_sle_solver.c \
	./src/fixed_rotation_constraint.c \
	./src/line_constraint.c \
	./src/constant_rotation_constraint.c \
	./src/link_constraint.c \
	./src/sparse_matrix.c \
	./src/constant_speed_motor.c \
	./src/gauss_seidel_sle_solver.c \
	./src/matrix.c \
	./src/rk4_ode_solver.c \
	./src/spring.c \
	./src/gaussian_elimination_sle_solver.c \
	./src/nsv_ode_solver.c \
	./src/rolling_constraint.c \
	./src/static_force_generator.c \
	./src/euler_ode_solver.c \
	./src/generic_rigid_body_system.c \
	./src/rotation_friction_constraint.c \
	./src/system_state.c \
	-I include

done:
	@echo
	@echo
	@echo Success; Done - fertig (ziemlich)
	@echo
	@echo see ./bin directory for output executable!
	@echo
