/*
FILENAME... NV40MotorDriver.cpp
USAGE...    


*/

#include "NV40.h"

static ELLLIST NV40List;
static int NV40ListInitialized = 0;

bool addToList(const char *portName, NV40Controller *drv) {
    if (!NV40ListInitialized) {
        NV40ListInitialized = 1;
        ellInit(&NV40List);
    } else if (findByPortName(portName) != NULL) {
        fprintf(stderr, "ERROR: Re-using portName=%s\n", portName);
        return false;
    }

    NV40Node *pNode = (NV40Node*)calloc(1, sizeof(NV40Node));
    pNode->portName = epicsStrDup(portName);
    pNode->pController = drv;
    ellAdd(&NV40List, (ELLNODE*)pNode);
    return true;
}

NV40Controller* findByPortName(const char *portName) {
    NV40Node *pNode;
    static const char *functionName = "findByPortName";

    // Find this 
    if (!NV40ListInitialized) {
        printf("%s:%s: ERROR, NV40 list not initialized\n",
            driverName, functionName);
        return NULL;
    }

    pNode = (NV40Node*)ellFirst(&NV40List);
    while(pNode) {
        if (!strcmp(pNode->portName, portName)) {
            return pNode->pController;
        }
        pNode = (NV40Node*)ellNext((ELLNODE*)pNode);
    }

    printf("%s: NV40 on port %s not found\n",
        driverName, portName);
    return NULL;
}

///// NV40CreateController
//
/** Creates a new NV40Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] type              The type of the controller [Use GCS for fully GCS-compatible controllers] (GCS, E-755, ...)
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] NV40PortName      The name of the drvAsynIPPPort that was created previously to connect to the NV40 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] pollPeriod        The time in ms between polls
  */
extern "C" int NV40CreateController(const char *portName, const char *NV40PortName, int numAxes, 
                                   int pollPeriod)
{
  new NV40Controller(portName, NV40PortName, numAxes, pollPeriod/1000.);
  return(asynSuccess);
}


/** Code for iocsh registration */
static const iocshArg NV40CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg NV40CreateControllerArg1 = {"NV40 port name", iocshArgString};
static const iocshArg NV40CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg NV40CreateControllerArg3 = {"Poll period (ms)", iocshArgInt};
static const iocshArg * const NV40CreateControllerArgs[] = {&NV40CreateControllerArg0,
                                                               &NV40CreateControllerArg1,
                                                               &NV40CreateControllerArg2,
                                                               &NV40CreateControllerArg3
                                                            };
static const iocshFuncDef NV40CreateControllerDef = {"NV40CreateController", 4, NV40CreateControllerArgs};
static void NV40CreateControllerCallFunc(const iocshArgBuf *args)
{
  NV40CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}

/***********************************************************************/
static void NV40MotorRegister(void)
{
  iocshRegister(&NV40CreateControllerDef, NV40CreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(NV40MotorRegister);
}

/*
//TODO: remove
#include <asynPortDriver.h>
asynStatus asynPortDriver::clearUInt32DigitalInterrupt(int, unsigned int) { return asynSuccess; }
asynStatus asynPortDriver::setUInt32DigitalInterrupt(int, unsigned int, interruptReason) { return asynSuccess; }
asynStatus asynPortDriver::setUInt32DigitalInterrupt(int, int, unsigned int, interruptReason) { return asynSuccess; }
asynStatus asynPortDriver::clearUInt32DigitalInterrupt(int, int, unsigned int) { return asynSuccess; }
asynStatus asynPortDriver::getUInt32DigitalInterrupt(int, int, unsigned int*, interruptReason) { return asynSuccess; }
asynStatus asynPortDriver::getUInt32DigitalInterrupt(int, unsigned int*, interruptReason) { return asynSuccess; }
*/
