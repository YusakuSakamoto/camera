.PHONY:install
.PHONY:remove
all:
	cd install && make

install:
	cd install && sh install.sh 

remove:
	cd install && make clean

