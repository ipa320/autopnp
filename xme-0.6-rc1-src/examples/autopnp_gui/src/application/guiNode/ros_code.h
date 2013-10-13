#ifndef ROSCODE_H
#define ROSCODE_H

//typedef void (*displayImageFunction)(unsigned int, unsigned int, unsigned int, unsigned char*);

#include <vector>

class RosInit
{
public:
	explicit RosInit(int argc, char *argv[], void (*displayImageFunction)(unsigned int, unsigned int, unsigned int, const std::vector<unsigned char>&));
	~RosInit();
};

#endif // ROSCODE_H
