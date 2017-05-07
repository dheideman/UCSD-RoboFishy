/*****************************************************************************
 * V4L2Control.cpp
 *
 * C++ wrapper for accessing V4L2 driver settings.
 * 
 * Author: Daniel Heideman
 ******************************************************************************/

#include "V4L2Control.h"

using namespace std;

//////////////////
// Constructors //
//////////////////

/*******************************************************************************
 * V4L2Control()
 * 
 * Default constructor
 ******************************************************************************/
V4L2Control::V4L2Control()
{
  this->initialized = 0;
}

/*******************************************************************************
 * V4L2Control(char* device)
 * 
 * Constructor
 * Try to open the device
 ******************************************************************************/
V4L2Control::V4L2Control(char* device)
{
  this->fd = ::open(device,  O_RDWR /* required */ | O_NONBLOCK, 0);
  if (this->fd == -1)
  {
    cout << "Error: Could not open device \"" << device << "\"" << endl;
    this->initialized = 0;
  }
  else
  {
    this->initialized = 1;
  }
}

/*******************************************************************************
 * void ~V4L2Control()
 * 
 * Destructor
 ******************************************************************************/
V4L2Control::~V4L2Control()
{
  // Not sure what to do here, so as to not mess everything up, it is empty.
}

///////////////////////
// Device Open/Close //
///////////////////////

/*******************************************************************************
 * int open(char* device)
 * 
 * Try to open the device
 ******************************************************************************/
int  V4L2Control::open(char* device)
{
  // If it's already opened with a file, close it and say we're not ready yet.
  if (this->initialized)
  {
    this->initialized = 0;
    this->close();
  }
  
  // Now go and open the requested file.
  this->fd = ::open(device,  O_RDWR /* required */ | O_NONBLOCK, 0);
  if (this->fd == -1)
  {
    cout << "Error: Could not open device \"" << device << "\"" << endl;
    this->initialized = 0;
    return -1;
  }
  else
  {
    this->initialized = 1;
    return 0;
  }
}

/*******************************************************************************
 * int isOpened()
 * 
 * Return initialized state (1 if device opened, 0 if not)
 ******************************************************************************/
int  V4L2Control::isOpened()
{
  return this->initialized;
}

/*******************************************************************************
 * int close()
 * 
 * Close the attached device.  Returns -1 if could not close.
 ******************************************************************************/
int  V4L2Control::close()
{
  // Check if we already have a device open. If not, don't bother closing.
  if (this->initialized)
  {
    // Try closing device.
    if( ::close(fd)==-1)
    {
      // Device closing failed
      cout << "Error closing V4L2 device" << endl;
      return -1;
    }
    else
    {
      // Successfully closed file
      return 0;
    }
  }
  else
  {
    // Nothing to close, so we're all good
    return 0;
  }
}

/////////////////////////
// Setters and Getters //
/////////////////////////

/*******************************************************************************
 * int set(unsigned int id, int value)
 * 
 * Set the V4L2 control "id" to "value"
 ******************************************************************************/
int  V4L2Control::set(unsigned int id, int value)
{
  struct v4l2_control control;
  
  // Make sure we're initialized first
  if(this->initialized)
  {
    memset(&control, 0, sizeof (control));
    control.id = id;
    control.value = value;
    if (-1 == ioctl(this->fd, VIDIOC_S_CTRL, &control))
    {
      this->writeError(id, value);
      return -1;
    }
    else return 0;
  }
  else return -1;
}

/*******************************************************************************
 * int get(unsigned int id)
 * 
 * Returns the current value of the V4L2 control "id"
 ******************************************************************************/
int  V4L2Control::get(unsigned int id)
{
  struct v4l2_control control;
  
  // Make sure we're initialized first
  if(this->initialized)
  {
    memset(&control, 0, sizeof (control));
    control.id = id;
    if (-1 == ioctl(this->fd, VIDIOC_G_CTRL, &control)) return -1;
    else return control.value;
  }
  else return -1;
}

///////////////////////
// Private Functions //
///////////////////////

/*******************************************************************************
 * void writeError(unsigned int id, int value)
 * 
 * Write to terminal the details of an error given the id trying to be changed
 * and the value it was to be set to.
 ******************************************************************************/
void V4L2Control::writeError(unsigned int id, int value)
{
  struct v4l2_querymenu qmenu;
  struct v4l2_queryctrl qctrl;
  
  string ctrlname;
  string menuvalue;
  int namelength;
  
  // make sure qctrl is cleared
  memset(&qctrl, 0, sizeof(qctrl));
  
  // Find the control that matches the requested id
  qctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
  while ((ctrlname.size()==0) && (0==ioctl(this->fd, VIDIOC_QUERYCTRL, &qctrl)))
  { 
    if( qctrl.id == id)
    {
      // Save the name of the control (as a string)
      namelength = sizeof qctrl.name / sizeof qctrl.name[0];
      ctrlname = string(qctrl.name, qctrl.name + namelength);
      
      // Check if this control is a menu type
      if (qctrl.type == V4L2_CTRL_TYPE_MENU)
      {
        memset(&qmenu, 0, sizeof(qmenu));
        qmenu.id = qctrl.id;
        
        // Loop through the menu items
        for (qmenu.index = qctrl.minimum;
             qmenu.index <= qctrl.maximum;
             qmenu.index++)
        {
          if (0 == ioctl(this->fd, VIDIOC_QUERYMENU, &qmenu))
          {
            if(qmenu.id == value)
            {
              namelength = sizeof qmenu.name / sizeof qmenu.name[0];
              menuvalue = string(qmenu.name, qmenu.name + namelength);
            }
          }
        } // end for (qmenu.index...)
      } // end if(qctrl.type...)        
    } // end if( qctrl.id...)
    
    // Move to next id.
    qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
  } // end while
  
  // Write out the error message
  // if a name was found for the value setting:
  if (menuvalue.size() != 0)
    cout << "Could not set \"" << ctrlname << "\" to " << menuvalue << endl;
  // otherwise just use the value given to us
  else
    cout << "Could not set \"" << ctrlname << "\" to " << value << endl;
    
  // Print out error code:
  switch (errno)
  {
    case EINVAL:
      cout << "Error Code: EINVAL" << endl;
      break;
    case ERANGE:
      cout << "Error Code: ERANGE" << endl;
      break;
    case EBUSY:
      cout << "Error Code: EBUSY" << endl;
      break;
    case EACCES:
      cout << "Error Code: EACCES" << endl;
      break;
  }
}
