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

THISLIB=edgegen.a
LIBSRCS=edgegen.c  xnear.c    delaunay.c mlinkern.c

ALLSRCS=eg_main.c  $(LIBSRCS)

LIBS=$(BLDROOT)/KDTREE/kdtree.a   $(BLDROOT)/FMATCH/fmatch.a \
     $(BLDROOT)/LINKERN/linkern.a $(BLDROOT)/UTIL/util.a

all: edgegen $(THISLIB)

everything: all

edgegen: eg_main.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ eg_main.$o $(THISLIB) $(LIBS) $(LIBFLAGS)

clean:
	-rm -f *.$o $(THISLIB) edgegen

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

delaunay.$o: delaunay.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/delaunay.h $(I)/macrorus.h 
edgegen.$o:  edgegen.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/edgegen.h  $(I)/kdtree.h   $(I)/linkern.h  $(I)/fmatch.h   \
        $(I)/delaunay.h $(I)/mlinkern.h $(I)/macrorus.h 
eg_main.$o:  eg_main.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/edgegen.h  
mlinkern.$o: mlinkern.c $(I)/machdefs.h $(I2)/config.h  $(I)/kdtree.h   \
        $(I)/util.h     $(I)/mlinkern.h 
xnear.$o:    xnear.c    $(I)/machdefs.h $(I2)/config.h  $(I)/edgegen.h  \
        $(I)/util.h     $(I)/macrorus.h 
