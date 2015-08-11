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
CFLAGS = -g -O2  -I$(BLDROOT)/INCLUDE -I$(CCINCDIR) -I$(srcdir)
LDFLAGS = -g -O2 
LIBFLAGS = -lm 
RANLIB = ranlib
LPSOLVER_LIB = 

OBJ_SUFFIX = o
o = $(OBJ_SUFFIX)

THISLIB=localcut.a
LIBSRCS=localcut.c   chunks.c   first.c      lift.c  tsporacl.c \
        intmat.c     separate.c
ALLSRCS=loc_main.c first_main.c checker.c peeler.c peeler2.c $(LIBSRCS)

LIBS=$(BLDROOT)/CUT/cut.a           $(BLDROOT)/TINY/tiny.a     \
     $(BLDROOT)/LP/lp.a             $(BLDROOT)/BIGGUY/bigguy.a \
     $(BLDROOT)/UTIL/util.a

LIBS2=$(BLDROOT)/TSP/tsp.a           $(BLDROOT)/PQ/pq.a           \
      $(BLDROOT)/EDGEGEN/edgegen.a   $(BLDROOT)/LINKERN/linkern.a \
      $(BLDROOT)/VERIFY/verify.a     $(BLDROOT)/FMATCH/fmatch.a   \
      $(BLDROOT)/KDTREE/kdtree.a     $(BLDROOT)/COMBS/combs.a     \
      $(BLDROOT)/TINY/tiny.a         $(BLDROOT)/LP/lp.a           \
      $(BLDROOT)/BIGGUY/bigguy.a     $(BLDROOT)/UTIL/util.a       \
      $(BLDROOT)/HELDKARP/heldkarp.a $(BLDROOT)/CUT/cut.a \
      $(BLDROOT)/TSP/tsp.a           $(BLDROOT)/CUT/cut.a


all: localcut $(THISLIB)

everything: all checker first peeler peeler2

first: first_main.$o $(THISLIB) $(LIBS2)
	$(CC) $(LDFLAGS) -o $@ first_main.$o $(THISLIB) $(LIBS2) $(LPSOLVER_LIB) $(LIBFLAGS)

localcut: loc_main.$o $(THISLIB) $(LIBS2)
	$(CC) $(LDFLAGS) -o $@ loc_main.$o $(THISLIB) $(LIBS2) $(LPSOLVER_LIB) $(LIBFLAGS)

checker: checker.$o
	$(CC) $(LDFLAGS) -o $@ checker.$o $(LIBFLAGS)

peeler: peeler.$o $(BLDROOT)/UTIL/util.a
	$(CC) $(LDFLAGS) -o $@ peeler.$o $(BLDROOT)/UTIL/util.a $(LIBFLAGS)

peeler2: peeler2.$o $(THISLIB) $(LIBS2) $(LPSOLVER_LIB)
	$(CC) $(LDFLAGS) -o $@ peeler2.$o $(THISLIB) $(LIBS2) $(LPSOLVER_LIB) $(LIBFLAGS)

rays: rays.$o $(THISLIB) $(LIBS2)
	$(CC) $(LDFLAGS) -o $@ rays.$o $(THISLIB) $(LIBS2) $(LPSOLVER_LIB) $(LIBFLAGS)

clean:
	-rm -f *.$o $(THISLIB) first checker peeler peeler2 localcut

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

checker.$o:  checker.c  $(I)/machdefs.h $(I2)/config.h  
chunks.$o:   chunks.c   $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    $(I)/localcut.h \
        $(I)/macrorus.h 
first.$o:    first.c    $(I)/machdefs.h $(I2)/config.h  $(I)/localcut.h \
        $(I)/tsp.h      $(I)/util.h     $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    \
        $(I)/macrorus.h 
first_main.$o: first_main.c $(I)/machdefs.h $(I2)/config.h  $(I)/localcut.h \
        $(I)/tsp.h      $(I)/util.h     $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
intmat.$o:   intmat.c   $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/localcut.h $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    \
        $(I)/macrorus.h 
lift.$o:     lift.c     $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/localcut.h $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    \
        $(I)/macrorus.h 
loc_main.$o: loc_main.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/kdtree.h   $(I)/combs.h    $(I)/localcut.h 
localcut.$o: localcut.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    $(I)/localcut.h 
peeler.$o:   peeler.c   $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     
peeler2.$o:  peeler2.c  $(I)/machdefs.h $(I2)/config.h  $(I)/localcut.h \
        $(I)/tsp.h      $(I)/util.h     $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
separate.$o: separate.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/lp.h       $(I)/localcut.h $(I)/tsp.h      $(I)/edgegen.h  \
        $(I)/bigguy.h   $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
tsporacl.$o: tsporacl.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/localcut.h $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    \
        $(I)/tinytsp.h  
