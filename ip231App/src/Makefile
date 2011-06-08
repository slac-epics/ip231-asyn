TOP=../..

include $(TOP)/configure/CONFIG

LIBRARY_IOC_vxWorks += ip231
LIBRARY_IOC_RTEMS   += ip231

DBD += ip231.dbd

ip231_SRCS += drvIP231.cpp

ip231_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

