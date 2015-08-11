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

OBJ_SUFFIX = o
o = $(OBJ_SUFFIX)

THISLIB=fmatch.a
LIBSRCS=fmatch.c
ALLSRCS=fm_main.c $(LIBSRCS)

LIBS=$(BLDROOT)/EDGEGEN/edgegen.a $(BLDROOT)/KDTREE/kdtree.a \
     $(BLDROOT)/LINKERN/linkern.a $(BLDROOT)/UTIL/util.a

all: fmatch $(THISLIB)

everything: all

fmatch: fm_main.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ fm_main.$o $(THISLIB) $(LIBS) $(LIBFLAGS)

clean:
	-rm -f *.$o $(THISLIB) fmatch

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

fm_main.$o:  fm_main.c  $(I)/machdefs.h $(I2)/config.h  $(I)/fmatch.h   \
        $(I)/util.h     $(I)/kdtree.h   $(I)/edgegen.h  
fmatch.$o:   fmatch.c   $(I)/machdefs.h $(I2)/config.h  $(I)/fmatch.h   \
        $(I)/util.h     $(I)/kdtree.h   $(I)/macrorus.h 
