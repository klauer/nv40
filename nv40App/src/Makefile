TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += nv40

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install nv40.dbd into <top>/dbd
DBD += nv40.dbd

# specify all source files to be compiled and added to the library
nv40_SRCS += main.cpp NV40.cpp 

mc100_LIBS += motor asyn
mc100_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

