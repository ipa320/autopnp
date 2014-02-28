#ifndef ROSCODE_H
#define ROSCODE_H

//typedef void (*displayImageFunction)(unsigned int, unsigned int, unsigned int, unsigned char*);

#include <vector>

class RosInit
{
public:
	explicit RosInit();
	void init(int argc, char *argv[], void (*displayImageFunction)(unsigned int, unsigned int, unsigned int, const std::vector<unsigned char>&));
	~RosInit();

	int terminateRosThread;
};

static RosInit rosInit;

#endif // ROSCODE_H
