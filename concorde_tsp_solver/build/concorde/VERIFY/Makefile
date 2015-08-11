# Generated automatically from Makefile.in by configure.
#
#   This file is part of CONCORDE
#
#   (c) Copyright 1995--1999 by David Applegate, Robert Bixby,
#   Vasek Chvatal, and William Cook
#
#   Permission is granted for academic research use.  For other uses,
#   contact the authors for licensing options.
#
#   Use at your own risk.  We make no guarantees about the
#   correctness or usefulness of this code.
#


SHELL = /bin/sh
SRCROOT = ..
BLDROOT = ..
CCINCDIR=$(SRCROOT)/INCLUDE

srcdir = .

CC = gcc
CFLAGS = -g -O2  -I$(BLDROOT)/INCLUDE -I$(CCINCDIR)
LDFLAGS = -g -O2 
LIBFLAGS = -lm 
RANLIB = ranlib
LPSOLVER_LIB = 

OBJ_SUFFIX = o
o = $(OBJ_SUFFIX)

THISLIB=verify.a
LIBSRCS=verify.c
ALLSRCS=ver_main.c $(LIBSRCS)

LIBS=$(BLDROOT)/TSP/tsp.a           $(BLDROOT)/LOCALCUT/localcut.a   \
     $(BLDROOT)/TINY/tiny.a         $(BLDROOT)/EDGEGEN/edgegen.a   \
     $(BLDROOT)/LINKERN/linkern.a   $(BLDROOT)/CUT/cut.a           \
     $(BLDROOT)/COMBS/combs.a       $(BLDROOT)/FMATCH/fmatch.a     \
     $(BLDROOT)/BIGGUY/bigguy.a     $(BLDROOT)/KDTREE/kdtree.a     \
     $(BLDROOT)/PQ/pq.a                                            \
     $(BLDROOT)/HELDKARP/heldkarp.a $(BLDROOT)/LP/lp.a             \
     $(BLDROOT)/UTIL/util.a

all: $(THISLIB)

everything: all verify

verify: ver_main.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ ver_main.$o $(THISLIB) $(LIBS) $(LPSOLVER_LIB) $(LIBFLAGS)

isomorph: isomorph.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ isomorph.$o $(THISLIB) $(LIBS) $(LPSOLVER_LIB) $(LIBFLAGS)

clean:
	-rm -f *.$o $(THISLIB) verify isomorph

OBJS=$(LIBSRCS:.c=.o)

$(THISLIB): $(OBJS)
	$(AR) $(ARFLAGS) $(THISLIB) $(OBJS)
	$(RANLIB) $(THISLIB)

.PHONY: $(BLDROOT)/concorde.a
$(BLDROOT)/concorde.a: $(OBJS)
	$(AR) $(ARFLAGS) $(BLDROOT)/concorde.a $(OBJS)
	$(RANLIB) $(BLDROOT)/concorde.a

include ../INCLUDE/Makefile.common

# DO NOT DELETE THIS LINE -- make depend depends on it.

I=$(CCINCDIR)
I2=$(BLDROOT)/INCLUDE

ver_main.$o: ver_main.c $(I)/machdefs.h $(I2)/config.h  $(I)/tsp.h      \
        $(I)/util.h     $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    $(I)/verify.h   
verify.$o:   verify.c   $(I)/machdefs.h $(I2)/config.h  $(I)/tsp.h      \
        $(I)/util.h     $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    $(I)/verify.h   \
        $(I)/macrorus.h $(I)/heldkarp.h 
