/*

Shamelessly copied from James Strawson's balance code

https://github.com/StrawsonDesign/Robotics_Cape_Installer/blob/64c324db4f5c33d9148759826837b84f4c709520/libraries/other/rc_time.c
 */
#include "Mapper.h"
#include <time.h>

/*******************************************************************************
* @ void auv_nanosleep(uint64_t ns)
*
* A wrapper for the normal UNIX nanosleep function which takes a number of
* nanoseconds instead of a timeval struct. This also handles restarting
* nanosleep with the remaining time in the event that nanosleep is interrupted
* by a signal. There is no upper limit on the time requested.
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
*******************************************************************************/
void auv_usleep(unsigned int us){
	auv_nanosleep(us*1000);
	return;
}
