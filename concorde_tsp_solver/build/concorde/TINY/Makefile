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

THISLIB=tiny.a
LIBSRCS=tinytsp.c  bnbmsp.c
ALLSRCS=tt_main.c  $(LIBSRCS)

LIBS=$(BLDROOT)/BIGGUY/bigguy.a  \
     $(BLDROOT)/LP/lp.a              $(BLDROOT)/CUT/cut.a         \
     $(BLDROOT)/TSP/tsp.a            $(BLDROOT)/EDGEGEN/edgegen.a \
     $(BLDROOT)/KDTREE/kdtree.a                                   \
     $(BLDROOT)/COMBS/combs.a        $(BLDROOT)/FMATCH/fmatch.a   \
     $(BLDROOT)/UTIL/util.a          $(BLDROOT)/PQ/pq.a           \
     $(BLDROOT)/LINKERN/linkern.a    $(BLDROOT)/CUT/cut.a         \
     $(BLDROOT)/LOCALCUT/localcut.a    $(BLDROOT)/VERIFY/verify.a   \
     $(BLDROOT)/HELDKARP/heldkarp.a

all: $(THISLIB)

everything: all tinytsp

tinytsp: tt_main.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ tt_main.$o $(THISLIB) $(LIBS) $(LPSOLVER_LIB) $(LIBFLAGS)

clean:
	-rm -f *.$o $(THISLIB) tinytsp

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

benttsp.$o:  benttsp.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tinytsp.h  $(I)/macrorus.h 
bnbmsp.$o:   bnbmsp.c   $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tinytsp.h  
bnbtsp.$o:   bnbtsp.c   $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tinytsp.h  
tinytsp.$o:  tinytsp.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tinytsp.h  $(I)/lp.h       $(I)/bigguy.h   $(I)/cut.h      \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/kdtree.h   $(I)/combs.h    
tt_main.$o:  tt_main.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tinytsp.h  
