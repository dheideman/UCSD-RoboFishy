/*****************************************************************************
 * Core.cpp
 *
 * Core Module: Defines global submersible types
 * 
 ******************************************************************************/

#include "Core.h"

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