CC = gcc
CFLAGS = -I/home/ghan/study_ws/epos4_etherCAT/include
LDFLAGS = -L/opt/etherlab/lib -lethercat

SRC = src/start_code_epos4_ecat.c
OBJ = epos4_01

all: $(OBJ)

$(OBJ): $(SRC)
	$(CC) -o $(OBJ) $(SRC) $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(OBJ)