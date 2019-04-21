SOURCES=$(wildcard *.c)
OBJECTS=$(addprefix .obj/,$(SOURCES:.c=.o))
CFLAGS=-D_GNU_SOURCE -Wall -Werror -pedantic -std=c99 -g -O2
LDFLAGS=-lm -lpthread

.obj:
	mkdir -p .obj

.obj/%.o: %.c *.h .obj
	gcc $(CFLAGS) -c $< -o $@

raspberryegg: $(OBJECTS)
	gcc $(CFLAGS) $(LDFLAGS) $(OBJECTS) -o $@

clean:
	rm $(OBJECTS) raspberryegg
