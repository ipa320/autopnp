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

THISLIB=tsp.a

LIBSRCS=prob_io.c  tsp_lp.c   qsparse.c  cliqhash.c generate.c             \
        ex_price.c cutcall.c  tighten.c  cutpool.c  branch.c   control.c   \
        cliqwork.c ddecker.c  teething.c bcontrol.c xtour.c    skeleton.c  \
        blkcomb.c  growcomb.c combcliq.c prclique.c blossom.c  tsp_call.c
ALLSRCS=concorde.c poolcat.c  bosstell.c cutserv.c  probserv.c test_tsp.c  \
        $(LIBSRCS)
LIBS=$(BLDROOT)/PQ/pq.a             $(BLDROOT)/VERIFY/verify.a   \
     $(BLDROOT)/LOCALCUT/localcut.a   $(BLDROOT)/TINY/tiny.a       \
     $(BLDROOT)/LINKERN/linkern.a   $(BLDROOT)/EDGEGEN/edgegen.a \
     $(BLDROOT)/KDTREE/kdtree.a     $(BLDROOT)/FMATCH/fmatch.a   \
     $(BLDROOT)/UTIL/util.a                                      \
     $(BLDROOT)/BIGGUY/bigguy.a     $(BLDROOT)/CUT/cut.a         \
     $(BLDROOT)/HELDKARP/heldkarp.a $(BLDROOT)/LP/lp.a           \
     $(BLDROOT)/COMBS/combs.a

all: concorde $(THISLIB)

everything: all poolcat test_tsp probserv cutserv bosstell

test_tsp: test_tsp.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ test_tsp.$o $(THISLIB) $(LIBS) $(LPSOLVER_LIB) $(LIBFLAGS)

concorde: concorde.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ concorde.$o $(THISLIB) $(LIBS) $(LPSOLVER_LIB) $(LIBFLAGS)

probserv: probserv.$o $(THISLIB) $(BLDROOT)/LP/lp.a $(BLDROOT)/BIGGUY/bigguy.a \
                             $(BLDROOT)/UTIL/util.a
	$(CC) $(LDFLAGS) -o $@ probserv.$o $(THISLIB) $(BLDROOT)/LP/lp.a $(BLDROOT)/BIGGUY/bigguy.a $(BLDROOT)/UTIL/util.a $(LPSOLVER_LIB) $(LIBFLAGS)

cutserv: cutserv.$o $(THISLIB) $(BLDROOT)/UTIL/util.a
	$(CC) $(LDFLAGS) -o $@ cutserv.$o $(THISLIB) $(BLDROOT)/UTIL/util.a $(LIBFLAGS)

bosstell: bosstell.$o $(BLDROOT)/UTIL/util.a
	$(CC) $(LDFLAGS) -o $@ bosstell.$o $(BLDROOT)/UTIL/util.a $(LIBFLAGS)

poolcat: poolcat.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ poolcat.$o $(THISLIB) $(LIBS) $(LPSOLVER_LIB) $(LIBFLAGS)

subboss: subboss.$o $(BLDROOT)/UTIL/util.a
	$(CC) $(LDFLAGS) -o $@ subboss.$o $(BLDROOT)/UTIL/util.a $(LIBFLAGS)

subgate: subgate.$o $(BLDROOT)/UTIL/util.a
	$(CC) $(LDFLAGS) -o $@ subgate.$o $(BLDROOT)/UTIL/util.a $(LIBFLAGS)

subgrunt: subgrunt.$o $(THISLIB) $(LIBS)
	$(CC) $(LDFLAGS) -o $@ subgrunt.$o $(THISLIB) $(LIBS) $(BLDROOT)/LINKERN/linkern.a $(LPSOLVER_LIB) $(LIBFLAGS)


clean:
	-rm -f *.$o $(THISLIB) concorde cutserv probserv bosstell poolcat test_tsp

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

bcontrol.$o: bcontrol.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/edgegen.h  $(I)/tsp.h      $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
blkcomb.$o:  blkcomb.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/kdtree.h   $(I)/combs.h    
blossom.$o:  blossom.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/kdtree.h   $(I)/combs.h    
bosstell.$o: bosstell.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
branch.$o:   branch.c   $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/macrorus.h $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
cliqhash.$o: cliqhash.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
cliqwork.$o: cliqwork.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/macrorus.h $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
combcliq.$o: combcliq.c $(I)/machdefs.h $(I2)/config.h  $(I)/macrorus.h \
        $(I)/util.h     $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
concorde.$o: concorde.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/edgegen.h  $(I)/tsp.h      $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h                    \
        $(I)/linkern.h  $(I)/macrorus.h 
control.$o:  control.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    $(I)/pq.h       \
        $(I)/pqsets.h   $(I)/cuttree.h  $(I)/consec1.h  $(I)/necklace.h \
        $(I)/localcut.h 
cutcall.$o:  cutcall.c  $(I)/machdefs.h $(I2)/config.h  $(I)/macrorus.h \
        $(I)/util.h     $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    \
        $(I)/verify.h   
cutpool.$o:  cutpool.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/macrorus.h $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
cutserv.$o:  cutserv.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
ddecker.$o:  ddecker.c  $(I)/machdefs.h $(I2)/config.h  $(I)/macrorus.h \
        $(I)/util.h     $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    \
        $(I)/verify.h   
ex_price.$o: ex_price.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/macrorus.h $(I)/bigguy.h   $(I)/tsp.h      $(I)/edgegen.h  \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
generate.$o: generate.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/macrorus.h $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
growcomb.$o: growcomb.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/kdtree.h   $(I)/combs.h    
poolcat.$o:  poolcat.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
prclique.$o: prclique.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/cut.h      $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/kdtree.h   $(I)/combs.h    
prob_io.$o:  prob_io.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
probserv.$o: probserv.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
qsparse.$o:  qsparse.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
skeleton.$o: skeleton.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
teething.$o: teething.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
test_tsp.$o: test_tsp.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
tighten.$o:  tighten.c  $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/tsp.h      $(I)/edgegen.h  $(I)/bigguy.h   $(I)/lp.h       \
        $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
tsp_call.$o: tsp_call.c $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/macrorus.h $(I)/fmatch.h   $(I)/edgegen.h  $(I)/linkern.h  \
        $(I)/tsp.h      $(I)/bigguy.h   $(I)/lp.h       $(I)/cut.h      \
        $(I)/kdtree.h   $(I)/combs.h    $(I)/pq.h       $(I)/pqsets.h   \
        $(I)/cuttree.h  $(I)/verify.h   
tsp_lp.$o:   tsp_lp.c   $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/macrorus.h $(I)/fmatch.h   $(I)/edgegen.h  $(I)/linkern.h  \
        $(I)/tsp.h      $(I)/bigguy.h   $(I)/lp.h       $(I)/cut.h      \
        $(I)/kdtree.h   $(I)/combs.h    $(I)/pq.h       $(I)/pqsets.h   \
        $(I)/cuttree.h  $(I)/verify.h   
xtour.$o:    xtour.c    $(I)/machdefs.h $(I2)/config.h  $(I)/util.h     \
        $(I)/edgegen.h  $(I)/linkern.h  $(I)/tsp.h      $(I)/bigguy.h   \
        $(I)/lp.h       $(I)/cut.h      $(I)/kdtree.h   $(I)/combs.h    
