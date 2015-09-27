#! /bin/bash
ar r /usr/local/lib/sakamoto/libsakamoto.a Like_terminal.o
sudo cp *.h /usr/local/include/sakamoto/
sudo rm /usr/lib/pkgconfig/sakamoto.pc
sudo cp *.pc /usr/lib/pkgconfig/
sudo cp *.txt /usr/local/bin/sakamoto/
echo "installation complete"
