CC = gcc
LD = gcc
CFLAGS = -Wall -Werror -Wl,-rpath,/home/user/lib -I/home/user/include
LDFLAGS = -L/home/user/lib -lpulse -lpthread

EXE = adc2pulse

$(EXE): $(EXE).c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(EXE)
