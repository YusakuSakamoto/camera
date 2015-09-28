#include <Like_terminal.h>

int main(void)
{
  startup();
  int i=0;
  int name_size;
  int pwd_size;
  int flag;
  char pwd[PATH_SIZE];
  char input[500];

  getcwd(pwd,PATH_SIZE);

  flag=0;
  while(flag==0){

	cout << NAME_COLOR1 << "arc--->>";

	name_size = strlen( "arc--->>" );
	pwd_size = strlen(pwd);

	cout << NAME_COLOR2 << " ~";
	for(i=name_size; i<pwd_size ;i++){
	  printf("%c",pwd[i]);
	}
	printf(" $ \x1b[39m");

	gets(input);
	if(input[0]!='\0'){
	  flag=1;
	}
  }

  printf("%s\n",input);
  end();

  return 0;
}
