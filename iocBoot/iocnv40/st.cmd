#!../../bin/linux-x86_64/nv40test

< envPaths

## Register all support components
dbLoadDatabase("../../dbd/nv40test.dbd",0,0)
nv40test_registerRecordDeviceDriver(pdbbase) 

# Get the NV40 port from the environment variable settings (see st.sh)

epicsEnvSet("EPICS_CA_AUTO_ADDR_LIST", "NO")
epicsEnvSet("EPICS_CA_ADDR_LIST", "10.5.0.255")

epicsEnvSet("MOXA_IP" "10.5.2.55")
epicsEnvSet("MOXA_PORT" "4006")
epicsEnvSet("P" "NV40:")
epicsEnvSet("R" "")
epicsEnvSet("NV40_PORT" "NV40")
epicsEnvSet("AS_PREFIX", "$(P)$(R)")
epicsEnvShow "MOXA_IP"
epicsEnvShow "MOXA_PORT"

drvAsynIPPortConfigure("IP1" ,"$(MOXA_IP):$(MOXA_PORT)",0,0,0)
## 19200 8n1 xon/xoff

#NV40CreateController(portName, NV40PortName, numAxes, pollPeriod)
NV40CreateController("$(NV40_PORT)", "IP1", 3, 50)

#asynSetTraceMask("$(NV40_PORT)", 0, 9)
#asynSetTraceMask("IP1", -1, 9)
#
#asynSetTraceIOMask("$(NV40_PORT)", -1, 255)
#asynSetTraceIOMask("IP1", -1, 255)

dbLoadRecords("$(TOP)/db/nv40.db", "P=$(P),R=$(R),PORT=$(NV40_PORT),ADDR=0,TIMEOUT=0.1")
dbLoadTemplate("nv40.sub")

cd $(TOP)/iocBoot/$(IOC)
< save_restore.cmd
iocInit()

create_monitor_set("auto_positions.req", 5, "P=$(AS_PREFIX)")
create_monitor_set("auto_settings.req", 30, "P=$(AS_PREFIX)")
