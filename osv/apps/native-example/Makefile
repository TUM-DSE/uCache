.PHONY: module
module: hello #hello-static-pie hello-static-non-pie

CFLAGS = -std=gnu99 -fpie -rdynamic

hello: hello.c
	$(CC) -pie -o $@ $(CFLAGS) $(LDFLAGS) hello.c

hello-static-pie: hello.c
	$(CC) -fPIE -static-pie -o hello-static-pie hello.c

hello-static-non-pie: hello.c
	$(CC) -static -o hello-static-non-pie hello.c

clean:
	rm -f hello hello-static-pie hello-static-non-pie
