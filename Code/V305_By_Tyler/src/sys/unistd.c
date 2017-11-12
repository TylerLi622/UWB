/*! ----------------------------------------------------------------------------
 * @file	unistd.h
 * @brief	sleep implementation instead of library sleep
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#include "compiler.h"
#include "sleep.h"

#define _clock(x) clock(x)

unsigned __weak sleep(unsigned seconds)
{
	clock_t t0 = _clock();
	clock_t dt = seconds * CLOCKS_PER_SEC;

	while (_clock() - t0  < dt);
	return 0;
}

int __weak usleep(useconds_t useconds)
{
	clock_t t0 = _clock();
	clock_t dt = useconds / (1000000/CLOCKS_PER_SEC);

	while (_clock() - t0  < dt);
	return 0;
}
