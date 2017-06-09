/*****************************************************************************
 * Core.cpp
 *
 * Core Module: Defines global submersible types
 *
 ******************************************************************************/

#include "Core.h"
#include <time.h>
#include <errno.h>

//////////////////////
// Global Variables //
//////////////////////

// Global State Variable
sub_state_t substate;

// Thread attributes for different priorities
pthread_attr_t tattrlow, tattrmed, tattrhigh;


/*******************************************************************************
 * void initializeTAttr()
 *
 * Initialize PThread tattr: tattrlow, tattrmed, tattrhigh.
 ******************************************************************************/
void initializeTAttr()
{
  // Initialize scheduling parameters, priorities
  sched_param param;
  int policy, maxpriority;

  // Initialize priorities
  pthread_attr_init(&tattrlow);
  pthread_attr_init(&tattrmed);
  pthread_attr_init(&tattrhigh);

  // Get max priority
  pthread_attr_getschedpolicy(&tattrlow, &policy);
  maxpriority = sched_get_priority_max(policy);

  // Extract scheduling parameter
  pthread_attr_getschedparam (&tattrlow, &param);

  // Set up low priority
  param.sched_priority = maxpriority/4;
  pthread_attr_setschedparam (&tattrlow, &param);

  // Set up medium priority
  param.sched_priority = maxpriority/2;
  pthread_attr_setschedparam (&tattrmed, &param);

  // Set up high priority
  param.sched_priority = maxpriority-1;
  pthread_attr_setschedparam (&tattrhigh, &param);

  return;
}


/*******************************************************************************
 * void destroyTAttr()
 *
 * Destroy PThread tattr: tattrlow, tattrmed, tattrhigh.
 ******************************************************************************/
void destroyTAttr()
{
  pthread_attr_destroy(&tattrlow);
  pthread_attr_destroy(&tattrmed);
  pthread_attr_destroy(&tattrhigh);

  return;
}

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

/*******************************************************************************
* @ void auv_nanosleep(uint64_t ns)
*
* A wrapper for the normal UNIX nanosleep function which takes a number of
* nanoseconds instead of a timeval struct. This also handles restarting
* nanosleep with the remaining time in the event that nanosleep is interrupted
* by a signal. There is no upper limit on the time requested.
*
* Shamelessly copied from James Strawson's balance code
* https://github.com/StrawsonDesign/Robotics_Cape_Installer/blob/64c324db4f5c33d9148759826837b84f4c709520/libraries/other/rc_time.c
*******************************************************************************/
void auv_nanosleep(uint64_t ns){
	struct timespec req,rem;
	req.tv_sec = ns/1000000000;
	req.tv_nsec = ns%1000000000;

	// loop until nanosleep sets an error or finishes successfully
	errno=0; // reset errno to avoid false detection
	while(nanosleep(&req, &rem) && errno==EINTR){
		req.tv_sec = rem.tv_sec;
		req.tv_nsec = rem.tv_nsec;
	}
	return;
}

/*******************************************************************************
* @ void auv_usleep(uint64_t ns)
*
* The traditional usleep function, however common, is deprecated in linux as it
* uses SIGALARM which interferes with alarm and timer functions. This uses the
* new POSIX standard nanosleep to accomplish the same thing which further
* supports sleeping for lengths longer than 1 second. This also handles
* restarting nanosleep with the remaining time in the event that nanosleep is
* interrupted by a signal. There is no upper limit on the time requested.
*
* Shamelessly copied from James Strawson's balance code
* https://github.com/StrawsonDesign/Robotics_Cape_Installer/blob/64c324db4f5c33d9148759826837b84f4c709520/libraries/other/rc_time.c
*******************************************************************************/
void auv_usleep(unsigned int us){
	auv_nanosleep(us*1000);
	return;
}
