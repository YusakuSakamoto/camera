#include <iostream>

class RAList {
	// This is cut from Mean Shift Analysis Library, Implemented by Chris M. Christoudias, Bogdan Georgescu
public:
	int		label;
	RAList	*next;
	RAList( void );
	~RAList( void );
	int Insert(RAList*);

private:
	///////current and previous pointer/////
	RAList	*cur, *prev;
	unsigned char exists;
};
