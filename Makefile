.PHONY:install
.PHONY:remove
install:
	cd install && make && sh install.sh

remove:
	cd install && make clean && cd ../sample && make clean
