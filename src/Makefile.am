SUFFIXES =

SUBDIRS							= 

AM_CPPFLAGS						= 	

EXTRA_DIST 						= 

bins						= $(top_builddir)/bin

### bedasm.exe ###
bin_PROGRAMS                = $(bins)/ntpccomm

__bins__ntpccomm_SOURCES	=									\
								main.cpp						\
								Log.cpp							\
								Logger.cpp						\
								SerialComm.cpp					\
																\
								CommHandler.h					\
								Log.h							\
								Logger.h						\
								SerialComm.h					\
								SerialComm.hpp					

EXTRA___bins__ntpccomm_SOURCES	= 								\
									CommHandler-unix.cpp		\
									CommHandler-windows.cpp		
									

__bins__ntpccomm_LDADD			= $(COMMHANDLER_SYSTEM)
__bins__ntpccomm_DEPENDENCIES	= $(COMMHANDLER_SYSTEM)
__bins__ntpccomm_LDFLAGS		= 
							
CLEANFILES					= 

