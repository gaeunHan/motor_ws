CC = gcc
CFLAGS = -I/home/ghan/study_ws/epos4_etherCAT/include
LDFLAGS = -L/opt/etherlab/lib -lethercat -lm  # -lm: linking standard c math library

SRC1 = src/start_code_epos4_ecat.c
OBJ1 = epos4_01

SRC2 = src/two_motors_and_logging.c
OBJ2 = epos4_02

all: $(OBJ1) $(OBJ2)

$(OBJ1): $(SRC1)
	$(CC) -o $(OBJ1) $(SRC1) $(CFLAGS) $(LDFLAGS)

$(OBJ2): $(SRC2)
	$(CC) -o $(OBJ2) $(SRC2) $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(OBJ1) $(OBJ2)
