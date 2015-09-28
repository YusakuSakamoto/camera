#! /bin/bash
sudo mkdir /usr/local/include/arc
sudo mkdir /usr/local/lib/arc
sudo mkdir /usr/local/bin/arc
ar r /usr/local/lib/arc/libarc.a arc.o
sudo cp *.h /usr/local/include/arc/
sudo rm /usr/lib/pkgconfig/arc.pc
sudo cp arc.pc /usr/lib/pkgconfig/
sudo cp *.txt /usr/local/bin/arc/
echo "installation complete"
