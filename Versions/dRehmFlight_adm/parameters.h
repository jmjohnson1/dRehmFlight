#ifndef PARAM_H
#define PARAM_H

// YOU MUST CHANGE THIS WHEN ADDING OR REMOVING PARAMETERS
// I do this because "new" scares me in embedded programs.
// Maybe this can change later.
#define NUMBER_OF_PARAMS 3

#include <stdint.h>

typedef struct param_s {
	char name[16];
	float value;
	uint8_t mavType;

	param_s(char parameterName[16], float parameterValue, uint8_t parameterType) {
		for (int i = 1; i < 15; ++i) {
			name[i] = parameterName[i];
		}
		value = parameterValue;
		mavType = parameterType;
	}
} param_t;

class ParameterTracker {
public:
	ParameterTracker();

private:
	param_t params[NUMBER_OF_PARAMS];
};

#endif //PARAM_H
