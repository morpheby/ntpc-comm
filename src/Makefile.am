SUFFIXES =

SUBDIRS							= 

AM_CPPFLAGS						= 	

EXTRA_DIST 						= 

CLEANFILES						= 

bins						= $(top_builddir)/bin
tests						= $(top_builddir)/tests

bin_PROGRAMS                = $(bins)/ntpccomm
noinst_LIBRARIES			= libntpccomm.a

__bins__ntpccomm_SOURCES		=									\
									main.cpp
__bins__ntpccomm_LDADD			= libntpccomm.a
__bins__ntpccomm_DEPENDENCIES	= libntpccomm.a
__bins__ntpccomm_LDFLAGS		= 

libntpccomm_a_SOURCES			=									\
									Log.cpp							\
									Logger.cpp						\
									SerialComm.cpp					\
									Atom.cpp						\
																	\
									CommHandler.h					\
									Log.h							\
									Logger.h						\
									SerialComm.h					\
									SerialComm.hpp					\
									Atom.h							\
									Atom.hpp						

EXTRA_libntpccomm_a_SOURCES	= 									\
									CommHandler-unix.cpp		\
									CommHandler-windows.cpp		
									

libntpccomm_a_LIBADD		= $(COMMHANDLER_SYSTEM)
libntpccomm_a_DEPENDENCIES	= $(COMMHANDLER_SYSTEM)


check_PROGRAMS				= $(tests)/serialcomm-test $(tests)/atom-test
TESTS						= $(tests)/atom-test

__tests__serialcomm_test_SOURCES		= SerialComm_test.cpp
__tests__serialcomm_test_LDADD			= libntpccomm.a
__tests__serialcomm_test_LDFLAGS		= 

__tests__atom_test_SOURCES				= Atom_test.cpp
__tests__atom_test_LDADD				= libntpccomm.a
__tests__atom_test_LDFLAGS				= 



