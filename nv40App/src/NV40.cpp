#include "NV40.h"

/** Creates a new NV40Axis object.
  * \param[in] controller         The NV40 controller
  * \param[in] axis_num           The axis number (1-based)
  */
NV40Axis::NV40Axis(NV40Controller *controller, int axis_num)
  :  asynMotorAxis((asynMotorController*)controller, axis_num)
{
    id_ = axis_num;
    pc_ = controller;
    encoderPos_ = 0.0;
    status_failed_ = 0;
}


bool NV40Axis::setRemoteControl(bool enabled) {
  asynStatus ret = pc_->write("setk,%d,%d", id_, (enabled ? 1 : 0));
 
  if (ret != asynSuccess)
    return false;
  else
    return pc_->check_error();
}

asynStatus NV40Axis::setClosedLoop(bool enabled) {
  asynStatus ret = pc_->write("chloop,%d,%d", id_, (enabled ? 1 : 0));
 
  if (ret != asynSuccess)
    return ret;
  else
    return (pc_->check_error() ? asynSuccess : asynError);
}

asynStatus NV40Axis::poll(bool *moving) {
  if (id_ == 0) {
    pc_->queryPositions();
  }
  //queryPosition();

  *moving = false;

  setIntegerParam(pc_->motorStatusMoving_, false);
  setIntegerParam(pc_->motorStatusDone_, true);

  callParamCallbacks();

  return asynSuccess;
}

/** Creates a new NV40Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] NV40PortName     The name of the drvAsynIPPPort that was created previously to connect to the NV40 controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
NV40Controller::NV40Controller(const char *portName, const char *NV40PortName, int numAxes,
                             double pollPeriod)
  :  asynMotorController(portName, numAxes, NUM_NV40_PARAMS,
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  NV40Axis *pAxis;
  static const char *functionName = "NV40Controller::NV40Controller";

  unitScale_ = NV40_UNIT_SCALE;
  if (!addToList(portName, this)) {
    printf("%s:%s: Init failed", driverName, portName);
    return;
  }

  movingPollPeriod_ = pollPeriod;
  idlePollPeriod_ = pollPeriod;
  printf("Poll period: %f\n", pollPeriod);
  createParam(NV40_STATUS_STRING, asynParamOctet, &NV40StatusString_);

  setStringParam(NV40StatusString_, "Startup");
  timeout_ = NV40_TIMEOUT;

  /* Connect to NV40 controller */
  status = pasynOctetSyncIO->connect(NV40PortName, 0, &pasynUser_, NULL);
  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: cannot connect to NV40 controller\n",
      driverName, functionName);
  }

  status = pasynOctetSyncIO->setInputEos(pasynUser_, "\r", 1);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
      "%s: Unable to set input EOS on %s: %s\n",
      functionName, NV40PortName, pasynUser_->errorMessage);
  }

  status = pasynOctetSyncIO->setOutputEos(pasynUser_, "\r", 1);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
      "%s: Unable to set output EOS on %s: %s\n",
      functionName, NV40PortName, pasynUser_->errorMessage);
  }

  queryVersion();

  // Create the axis objects
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new NV40Axis(this, axis);
    pAxis->setRemoteControl(true);
  }
  startPoller(pollPeriod/1000., pollPeriod/1000., 2);
}

asynStatus NV40Controller::queryVersion() {
  size_t num_read=0;
  char ver_string[NV40_STRING_SIZE];
  static const char* functionName = "NV40Controller::queryVersion";
  asynStatus ret = writeRead(ver_string, &num_read, "ver");
  if (num_read > 0 && ret == asynSuccess) {
    printf("%s: NV40 version: %s\n", functionName, ver_string);
  }
  return ret;
}

asynStatus NV40Controller::write(const char *fmt, va_list argptr) {
  size_t nwrite;
  asynStatus status;
  const char *functionName="write";
  const int buf_size = NV40_STRING_SIZE;
  char buf[buf_size];

  vsnprintf(buf, buf_size, fmt, argptr);

  lock();

#if DEBUG
  fprintf(stderr, "%s:%s: %s\n", driverName, functionName, buf);
#endif

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: %s\n",
    driverName, functionName, buf);

  status = pasynOctetSyncIO->write(pasynUser_,
                                   buf, strlen(buf),
                                   timeout_, &nwrite);
  unlock();

  return status;
}

asynStatus NV40Controller::write(const char *fmt, ...) {
  va_list argptr;
  va_start(argptr,fmt);
  asynStatus ret=write(fmt, argptr);
  va_end(argptr);
  return ret;
}

asynStatus NV40Controller::writeRead(char *input, size_t* nread, const char *fmt, va_list argptr) {

  size_t nwrite;
  asynStatus status;
  int eomReason;
  const char *functionName="writeRead";
  const int buf_size = NV40_STRING_SIZE;
  char buf[buf_size];

  vsnprintf(buf, buf_size, fmt, argptr);

  strncpy(outString_, buf, buf_size);
  lock();
#if DEBUG
  fprintf(stderr, "%s:%s: write: %s\n",
    driverName, functionName, buf);
#endif

  asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER,
    "%s:%s: Write: %s\n",
    driverName, functionName, buf);

  status = pasynOctetSyncIO->writeRead(pasynUser_,
                                       buf, strlen(buf),
                                       input, NV40_STRING_SIZE,
                                       timeout_, &nwrite, nread, &eomReason);

#if DEBUG
  fprintf(stderr, "%s:%s: Read (%db): %s\n",
    driverName, functionName, *nread, input);
#endif
  asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER,
    "%s:%s: Read (%db): %s\n",
    driverName, functionName, *nread, input);

  unlock();

  return status;
}

asynStatus NV40Controller::writeRead(char *input, size_t* nread, const char *fmt, ...) {
  va_list argptr;
  va_start(argptr,fmt);
  asynStatus ret=writeRead(input, nread, fmt, argptr);
  va_end(argptr);
  return ret;
}

#ifndef strnchr

char* strnchr(const char* str, size_t len, char c) {
  if (!str)
    return NULL;

  while (len > 0 && *str != '\0') {
    if (*str == c) {
      return (char*)str;
    }
    str++;
    len--;
  }
  return NULL;
}

#endif

asynStatus NV40Controller::queryPositions() {
  size_t num_read=0;
  char input[NV40_STRING_SIZE];
  double positions[NV40_AXES];
  char *pos_str[NV40_AXES];
  static const char* functionName = "NV40Controller::queryPositions";
  asynStatus ret = writeRead(input, &num_read, "measure");
  if (ret != asynSuccess || num_read < 10) // 10 == len("rk,n,x.xxx")
    return asynError;

  char *encoder_pos = input;

  // Format: aw,0.000,0.000,0.000
  // Find the commas, replace them with null terminators, and store the
  // start of the position strings.
  for (int i=0; i < NV40_AXES; i++) {
      encoder_pos = strnchr(encoder_pos+1, NV40_STRING_SIZE, ',');
      if (!encoder_pos) {
        return asynError;
      }

      encoder_pos[0] = 0;
      pos_str[i] = &encoder_pos[1];
  }

  for (int i=0; i < NV40_AXES; i++) {
    positions[i] = atof(pos_str[i]);
#if DEBUG
    fprintf(stderr, "positions[%d] = %g\n", i, positions[i]);
#endif
    if (i < numAxes_) {
        NV40Axis *axis=getAxis(i);
        axis->encoderPos_ = positions[i];
        axis->setDoubleParam(motorEncoderPosition_, positions[i] * unitScale_);
        axis->setDoubleParam(motorPosition_, positions[i] * unitScale_);
    }
  }

  return asynSuccess;
    
}

asynStatus NV40Axis::queryPosition() {

  size_t num_read=0;
  char input[NV40_STRING_SIZE];
  static const char* functionName = "NV40Axis::queryPosition";
  asynStatus ret = pc_->writeRead(input, &num_read, "rk,%d", id_);
  if (ret != asynSuccess || num_read < 10) // 10 == len("rk,n,xx.xxx")
    return asynError;

  // Format: rk,n,xx.xxx
  //           ^
  // So, find the comma, then the axis number and position are easily
  // separable
  char *encoder_pos=strnchr(input, NV40_STRING_SIZE, ',');
  if (!encoder_pos) {
    return asynError;
  }

  encoder_pos++;
  char axis=encoder_pos[0] - 0x30;
 
  if (axis != id_) {
    //printf("bad axis %d != %d\n", id_, axis);
    return asynError;
  }

  // jump past the comma
  encoder_pos += 2;

  encoderPos_ = atof(encoder_pos);
  setDoubleParam(pc_->motorEncoderPosition_, encoderPos_ * pc_->unitScale_);
  setDoubleParam(pc_->motorPosition_, encoderPos_ * pc_->unitScale_);

#if DEBUG
  fprintf(stderr, "encoder: %g\n" encoderPos_);
#endif
  return asynSuccess;
}

int
NV40Controller::check_error() {
  size_t read;
  int error=0;
  static const char* functionName = "NV40Controller::check_error";
  char input[NV40_STRING_SIZE];
  char *err_str=NULL;
  asynStatus status = writeRead(input, &read, "ERR?");
  if (status == asynSuccess && read > 0) {
    if (!strncmp(input, NV40_NO_ERROR, strlen(NV40_NO_ERROR))) {
      asynPrint(pasynUser_, ASYN_TRACE_FLOW,
        "%s:%s: success\n",
        driverName, functionName);
      setStringParam(NV40StatusString_, "");
    } else {
      //format ERROR,"(error string)"
      if (!strncmp(input, "ERROR,", strlen("ERROR,"))) {
        err_str = &input[6];
        asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
          "%s:%s: %s\n",
          driverName, functionName, err_str);

        err_str++; // leading quote
        if (!strncmp(err_str, NV40_ERRORS_PARAM, strlen(NV40_ERRORS_PARAM))) {
            err_str += strlen(NV40_ERRORS_PARAM);
        }

        if (strlen(err_str) > 40) {
          err_str[39] = '\0';
        }
        setStringParam(NV40StatusString_, err_str);
      }
      error = 1;
    }
  }
  return error;
}

asynStatus NV40Axis::stop(double acceleration)
{
  return asynSuccess;
}

asynStatus NV40Axis::moveVelocity(double min_velocity, double max_velocity, double acceleration) {
  // no such thing
  return asynError;
}

asynStatus NV40Axis::home(double min_velocity, double max_velocity, double acceleration, int forwards) {
  // no such thing
  return asynError;
}

asynStatus NV40Axis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
  static const char* functionName = "NV40Axis::move";
  asynStatus ret;

  position /= pc_->unitScale_;

  asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: axis %d: move to %g\n",
    driverName, functionName, id_, position);
  printf("%s:%s: axis %d: move to %g\n",
    driverName, functionName, id_, position);

  ret=pc_->write("set,%d,%.3f", id_, position);
  pc_->check_error();
  return ret;
}

