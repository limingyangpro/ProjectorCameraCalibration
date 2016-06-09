/*
 * chronometer.h
 *
 *  Created on: 11 déc. 2013
 *      Author: lyang
 */

#ifndef CHRONOMETER_H_
#define CHRONOMETER_H_

#include "basictype.h"
#include <time.h>
#include <ctime>

class Chronometer
{
private:
	clock_t t;
public:
	void tic();
	float tac() const;
	static string getTime();
};



#endif /* CHRONOMETER_H_ */
