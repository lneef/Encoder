SRC := $(wildcard lib/*.c)
obj-m += lib/encoder-driver.o lib/debounce-driver.o
PWD := $(shell pwd)

all: 
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
