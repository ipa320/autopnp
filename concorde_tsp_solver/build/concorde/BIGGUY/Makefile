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

THISLIB=bigguy.a
LIBSRCS=bigguy.c
ALLSRCS=bg_test.c $(LIBSRCS)

LIBS=$(BLDROOT)/UTIL/util.a

all: $(THISLIB)

everything: all bg_test test

bg_test: bg_test.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ bg_test.$o $(THISLIB) $(LIBS) $(LIBFLAGS)

test: bg_test
	time bg_test < $(srcdir)/bigguy.tst > bigguy.ts2
	cmp $(srcdir)/bigguy.tst bigguy.ts2

clean:
	-rm -f *.$o $(THISLIB) bg_test bigguy.ts2

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

bg_test.$o:  bg_test.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/bigguy.h   
bigguy.$o:   bigguy.c   $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/bigguy.h   
