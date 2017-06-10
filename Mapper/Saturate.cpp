/******************************************************************************
* Saturate.cpp
*
* Contains the saturate_number() function
******************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * int saturate_number(float* val, float min, float max)
 *
 * Generic function for saturating values in a function
***************************************************************************/
int saturate_number(float* val, float min, float max)
{
	// if "val" is greater than "max", set "val" to "max" //
	if(*val>max)
	{
		*val = max;
		return 1;
	}

	// if "val" is less than "min", set "val" to "min" //
	else if(*val<min)
	{
		*val = min;
		return 1;
	}
	return 0;
}