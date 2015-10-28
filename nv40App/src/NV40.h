#ifndef _NV40_H
#define _NV40_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>

#include <iocsh.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsExport.h>

#include <asynOctetSyncIO.h>
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define NV40_NO_ERROR             "ERROR,\"OK. No error.\""

#define NV40_TIMEOUT              0.5
#define NV40_STRING_SIZE          160 // no clue what error messages to expect...
#define NV40_UNIT_SCALE           1.0e6

#define NV40_AXES                 3
#define NV40_STATUS_STRING        "NV40_STATUS_STRING"

#define NV40_ERRORS_PARAM         "Parameter error in command: "

static const char *NV40_IGNORE_STRINGS[] = { "NV403CL>", NULL };

static const char* driverName = "NV40";
class NV40Controller;

class NV40Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  NV40Axis(NV40Controller *pC, int axis);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  //asynStatus setPosition(double position);

  /* And these are specific to this class: */
  bool setRemoteControl(bool enabled);
  asynStatus setClosedLoop(bool enabled);
  asynStatus queryPosition();

  inline bool isFlagSet(unsigned int flag) { return (flags_ & flag) == flag; }
  inline void setFlag(unsigned int flag)   { flags_ |= flag; }
  inline void clearFlag(unsigned int flag) { flags_ &= ~flag; }
  inline void setFlag(unsigned int flag, bool set) {
    if (set)
      flags_ |= flag;
    else
      flags_ &= ~flag;
  }

private:
  friend class NV40Controller;
  NV40Controller *pc_;      /**< Pointer to the asynMotorController to which this axis belongs.
                              *   Abbreviated because it is used very frequently */
  double encoderPos_;   /**< Cached copy of the encoder position */
  unsigned int flags_;       /**< Cached copy of the current flags */

  bool errored_;
  int axisNum_;         // according to asyn
  int id_;              // axis num according to controller
  int status_failed_;   // number of consecutive times status queries have failed

};

// if status queries fail (n) times in a row, restart the ioc
#define NV40_STATUS_FAILED_THRESHOLD 100

/* NOTE on unitScale_:
 * Trying to use 1.0 for the encoder/position resolution does not work,
 * regardless of if your encoder gives floating point position readout.
 *
 * It will result in truncation of the values in EPICS, such that you'd
 * only see 0.5, if in fact the motor was at 0.5345423. As such, the position
 * is kept properly internally, but it is scaled when being passed back
 * and forth from EPICS.
 */

class NV40Controller : public asynMotorController {
public:
  NV40Controller(const char *portName, const char *NV40PortName, int numAxes, double pollPeriod);
  NV40Axis* getAxis(int axisNo) {
    return (NV40Axis*)asynMotorController::getAxis(axisNo);
  }

  /* These are the methods that are new to this class */
  asynStatus write(const char* fmt, ...);
  asynStatus write(const char* fmt, va_list);
  asynStatus writeRead(char* input, size_t* nread, const char* fmt, ...);
  asynStatus writeRead(char* input, size_t* nread, const char* fmt, va_list);
  int getAxisCount() { return numAxes_; }
  asynStatus queryPositions();
  asynStatus queryVersion();

protected:
  int check_error();
#define FIRST_NV40_PARAM NV40StatusString_
  int NV40StatusString_;          // Jerk time parameter index
#define LAST_NV40_PARAM NV40StatusString_
#define NUM_NV40_PARAMS (&LAST_NV40_PARAM - &FIRST_NV40_PARAM + 1)
  double unitScale_; // see note above
  double timeout_;

private:
  friend class NV40Axis;
  asynUser *pasynUser_;
  char outString_[NV40_STRING_SIZE];

};

/* Use the following structure and functions to manage multiple instances
 * of the driver */
typedef struct NV40Node {
    ELLNODE node;
    const char *portName;
    NV40Controller *pController;
} NV40Node;

bool addToList(const char *portName, NV40Controller *drv);
NV40Controller* findByPortName(const char *portName);

#endif
