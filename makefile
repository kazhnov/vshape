DEBUG = -O0 -g -fsanitize=address
RELEASE = -O3
MODE = $(DEBUG)

subs:
	git submodule update --init --recursive
	git submodule foreach git pull origin main

build: vshape.c subs
	mkdir -p build lib
	cc -c vshape.c -o build/vshape.o $(MODE)
	ar rcs lib/libvshape.a build/vshape.o

test: test.c build
	cc test.c -o build/test -L./lib -lm -lvshape $(MODE) 
	build/test
