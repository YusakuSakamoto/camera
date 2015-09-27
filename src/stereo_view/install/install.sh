#! /bin/bash
ar r /usr/local/lib/arc/libarc.a Like_terminal.o
sudo cp *.h /usr/local/include/arc/
sudo rm /usr/lib/pkgconfig/arc.pc
sudo cp arc.pc /usr/lib/pkgconfig/
sudo cp *.txt /usr/local/bin/arc/
echo "installation complete"
