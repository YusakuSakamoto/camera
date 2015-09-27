#include "../include/calib.hpp"

int main(int argc, char *argv[])
{
  int status;
  pthread_t thread_a;
  pthread_t thread_c;
  void *thread_return;
  MY_THREAD_ARG thread_message;
  //calibrate();
	
  status = pthread_create(&thread_a, NULL, myThread, &thread_message);
  if(status != 0 )exit(1);
  status=pthread_create(&thread_c, NULL, myKey,NULL);
  if(status != 0 )exit(1);
  
  status = pthread_join(thread_c, &thread_return);
  if(status != 0 )exit(1);
  
  return 0;
}
