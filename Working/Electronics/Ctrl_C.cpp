/******************************************************************************
* Ctrl_C.cpp
*
* Contains the ctrl_c function
******************************************************************************/
#include "Mapper.h"

/***************************************************************************
 * void ctrl_c(int signo)
 *
 * Captures a user-entered ctrl+c and exits the script
***************************************************************************/
void ctrl_c(int signo)
{
	if (signo == SIGINT)
	{
		substate.mode = STOPPED;
		printf("\nReceived SIGINT Ctrl-C\n");
	}
}
