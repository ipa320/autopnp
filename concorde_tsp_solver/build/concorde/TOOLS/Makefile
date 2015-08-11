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

ALLSRCS=fconvert.c tourchk.c  \
        edgunion.c edg2len.c  showres.c  killgrun.c tourlen.c  \
        prob2tsp.c

LIBS=$(BLDROOT)/KDTREE/kdtree.a $(BLDROOT)/UTIL/util.a \
     $(BLDROOT)/EDGEGEN/edgegen.a                                 \
     $(BLDROOT)/LINKERN/linkern.a                                 \
     $(BLDROOT)/FMATCH/fmatch.a                                   \
     $(BLDROOT)/KDTREE/kdtree.a                                   \
     $(BLDROOT)/UTIL/util.a

all: showres tourchk tourlen prob2tsp edg2len edgunion fconvert

everything: all killgrun

fconvert: fconvert.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ fconvert.$o $(LIBS) $(LIBFLAGS)

tourchk: tourchk.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ tourchk.$o $(LIBS) $(LIBFLAGS)

showres: showres.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ showres.$o $(LIBS) $(LIBFLAGS)

killgrun: killgrun.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ killgrun.$o $(LIBS) $(LIBFLAGS)

prob2tsp: prob2tsp.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ prob2tsp.$o $(LIBS) $(LIBFLAGS)

tourlen: tourlen.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ tourlen.$o $(LIBS) $(LIBFLAGS)

edgunion: edgunion.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ edgunion.$o $(LIBS) $(LIBFLAGS)

edg2len: edg2len.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ edg2len.$o $(LIBS) $(LIBFLAGS)

tour2edg: tour2edg.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ tour2edg.$o $(LIBS) $(LIBFLAGS)

tdivide: tdivide.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ tdivide.$o $(LIBS) $(LIBFLAGS)

ttour: ttour.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ ttour.$o $(LIBS) $(LIBFLAGS)

lkhboss: lkhboss.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ lkhboss.$o $(LIBS) $(LIBFLAGS)

lkhgrunt: lkhgrunt.$o $(LIBS)
	$(CC) $(LDFLAGS) -o $@ lkhgrunt.$o $(LIBS) $(LIBFLAGS)


clean:
	-rm -f *.$o fconvert tourchk \
                    showres \
                    prob2tsp tourlen \
                    edgunion killgrun \
                    edg2len

$(BLDROOT)/concorde.a:

include ../INCLUDE/Makefile.common

# DO NOT DELETE THIS LINE -- make depend depends on it.

I=$(CCINCDIR)
I2=$(BLDROOT)/INCLUDE

edg2len.$o:  edg2len.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     
edgunion.$o: edgunion.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     
fconvert.$o: fconvert.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     
killgrun.$o: killgrun.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
prob2tsp.$o: prob2tsp.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     
showres.$o:  showres.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
tourchk.$o:  tourchk.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     
tourlen.$o:  tourlen.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     
tour2edg.$o: tour2edg.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     
tdivide.$o:  tdivide.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     
