#define __RASPBERRY_PI__

#ifdef __RASPBERRY_PI__
#define NAME_COLOR1 "\x1b[1m\x1b[32m"
#define NAME_COLOR2 "\x1b[1m\x1b[34m"
#endif

#ifndef __RASPBERRY_PI__
#define NAME_COLOR1 "\x1b[39m"
#define NAME_COLOR2 "\x1b[39m"
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
