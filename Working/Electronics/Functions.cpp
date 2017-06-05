#include "Mapper.h"

// state variable for loop and thread control //
enum state_t state = UNINITIALIZED;

/******************************************************************************
 * state_t get_state()
 *
 * Gets the AUV's current state
******************************************************************************/

state_t get_state()
{
	return state;
}

/******************************************************************************
 * int set_state(enum state_t new_state)
 *
 * Sets the AUV state
******************************************************************************/

int set_state(enum state_t new_state)
{
	state = new_state;
	return 0;
}
