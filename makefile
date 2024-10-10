CC = gcc
CFLAGS = -I/home/ghan/study_ws/epos4_etherCAT/include
LDFLAGS = -L/opt/etherlab/lib -lethercat -lm  # -lm: linking standard c math library

SRC1 = src/start_code_epos4_ecat.c
OBJ1 = epos4_01.out

SRC2 = src/two_motors_and_logging.c
OBJ2 = epos4_02.out

SRC3 = src/csv_with_pi_controller.c
OBJ3 = epos4_03.out

SRC4 = src/csp_poly5.c
OBJ4 = epos4_04.out

all: $(OBJ1) $(OBJ2) $(OBJ3) $(OBJ4)

$(OBJ1): $(SRC1)
	$(CC) -o $(OBJ1) $(SRC1) $(CFLAGS) $(LDFLAGS)

$(OBJ2): $(SRC2)
	$(CC) -o $(OBJ2) $(SRC2) $(CFLAGS) $(LDFLAGS)

$(OBJ3): $(SRC3)
	$(CC) -o $(OBJ3) $(SRC3) $(CFLAGS) $(LDFLAGS)

$(OBJ4): $(SRC4)
	$(CC) -o $(OBJ4) $(SRC4) $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(OBJ1) $(OBJ2) $(OBJ3) $(OJB4)
