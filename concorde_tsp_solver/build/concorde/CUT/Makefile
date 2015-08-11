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

THISLIB=cut.a
LIBSRCS=segments.c connect.c  shrink.c mincut.c cut_st.c gomoryhu.c
ALLSRCS=mc_main.c  lin_all.c  $(LIBSRCS)

LIBS=$(BLDROOT)/UTIL/util.a

all: mincut $(THISLIB)

everything: all lin_all

mincut: mc_main.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ mc_main.$o $(THISLIB) $(LIBS) $(LIBFLAGS)

lin_all: lin_all.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ lin_all.$o $(THISLIB) $(LIBS) $(LIBFLAGS)

clean:
	-rm -f *.$o $(THISLIB) mincut lin_all

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

connect.$o:  connect.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      
cut_st.$o:   cut_st.c   $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      
gomoryhu.$o: gomoryhu.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      
lin_all.$o:  lin_all.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      
mc_main.$o:  mc_main.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      
mincut.$o:   mincut.c   $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      
segments.$o: segments.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      
shrink.$o:   shrink.c   $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      
