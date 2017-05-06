/*****************************************************************************
 * V4L2Control.h
 *
 * C++ wrapper for accessing V4L2 driver settings.
 * 
 * Author: Daniel Heideman
 ******************************************************************************/
 
#ifndef V4L2Control_h
#define V4L2Control_h

#include <iostream>
#include <string>

// V4L2 Includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

class V4L2Control
{
  public:
    V4L2Control();
    V4L2Control(char* device);
    ~V4L2Control();
    int open(char* device);
    int isOpened();
    int close();
    int set(unsigned int id, int value);
    int get(unsigned int id);
    
  private:
    void writeError(unsigned int id, int value);
  
    int    fd;
    int    initialized;
};

#endif
