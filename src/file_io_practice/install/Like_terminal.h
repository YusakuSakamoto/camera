#define __RASPBERRY_PI__

#ifdef __RASPBERRY_PI__
#define NAME_COLOR1 "\x1b[1m\x1b[32m"
#define NAME_COLOR2 "\x1b[1m\x1b[34m"
#define NAME_STRING "pi@raspberrypi"
#define USER_NAME "/home/pi"
#endif

#ifndef __RASPBERRY_PI__
#define NAME_COLOR1 "\x1b[39m"
#define NAME_COLOR2 "\x1b[39m"
#define NAME_STRING "sakamoto@sakamoto-CF=AX2SDLTC"
#define USER_NAME "/home/sakamoto"
#endif

#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define STARTUP_FILE "startup.txt"
#define PATH_SIZE 512

using namespace std;

int startup(void);
int help(void);
int end(void);
