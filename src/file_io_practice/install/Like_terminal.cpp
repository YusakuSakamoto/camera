#include "Like_terminal.h"

int startup(void){
  ifstream ifs("/usr/local/bin/sakamoto/startup.txt");
  string str;
  if(ifs.fail()) return -1;
  while(getline(ifs,str)){
	cout << str << endl;
  }
  return 0;
}

int help(void){
  ifstream ifs("/usr/local/bin/sakamoto/help.txt");
  string str;
  if(ifs.fail()) return -1;
  while(getline(ifs,str))cout << str << endl;
  return 0;
}

int end(void){
  ifstream ifs("/usr/local/bin/sakamoto/end.txt");
  string str;
  if(ifs.fail()) return -1;
  while(getline(ifs,str))cout << str << endl;
  return 0;
}
