TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = nv40test
# nv40test.dbd will be created and installed
DBD += nv40test.dbd

# nv40test.dbd will be made up from these files:
nv40test_DBD += base.dbd

# Include dbd files from all support applications:
nv40test_DBD += drvAsynIPPort.dbd motorRecord.dbd motorSupport.dbd asyn.dbd nv40.dbd asSupport.dbd

# Add all the support libraries needed by this IOC
nv40test_LIBS += asyn nv40 motor autosave

# nv40test_registerRecordDeviceDriver.cpp derives from nv40test.dbd
nv40test_SRCS += nv40test_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
nv40test_SRCS_DEFAULT += nv40testMain.cpp
nv40test_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#nv40test_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
nv40test_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

