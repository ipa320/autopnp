/* INCLUDE/config.h.  Generated automatically by configure.  */
/****************************************************************************/
/*                                                                          */
/*  This file is part of CONCORDE                                           */
/*                                                                          */
/*  (c) Copyright 1995--1999 by David Applegate, Robert Bixby,              */
/*  Vasek Chvatal, and William Cook                                         */
/*                                                                          */
/*  Permission is granted for academic research use.  For other uses,       */
/*  contact the authors for licensing options.                              */
/*                                                                          */
/*  Use at your own risk.  We make no guarantees about the                  */
/*  correctness or usefulness of this code.                                 */
/*                                                                          */
/****************************************************************************/


/* Define if your compiler is missing the appropriate function prototype */

/* #undef CC_PROTO_PRINTF */
/* #undef CC_PROTO_GETHOSTNAME */
/* #undef CC_PROTO_GETRUSAGE */

/* Define if you want to use posix threads */
/* #undef CC_POSIXTHREADS */

/* Define if <signal.h> needs to be included before <pthreads.h> */
/* #undef CC_SIGNAL_BEFORE_PTHREAD */

/* Define to empty if the keyword `const' does not work.  */
/* #undef const */

/* Define to `int' if <sys/types.h> doesn't define.  */
/* #undef pid_t */

/* Define to `unsigned' if <sys/types.h> doesn't define.  */
/* #undef size_t */

/* Define to `unsigned char' if <sys/types.h> doesn't define.  */
/* #undef u_char */

/* Define to `int' if the builtin type `void' does not work.  */
/* #undef void */

/* Define as the return type of signal handlers (int or void).  */
#define RETSIGTYPE void

/* Define one of the following three to specify the type of signal
 * handling to use. */
#define  CCSIGNAL_SIGACTION 1 /* sigaction(), preferred */
/* #undef  CCSIGNAL_SIGNAL */    /* signal() */
/* #undef  CCSIGNAL_NONE */      /* no signal handling */

/* Define if you have the gethostname function.  */
#define HAVE_GETHOSTNAME 1

/* Define if you have the socket function.  */
#define HAVE_SOCKET 1

/* Define if you have the strdup function.  */
#define HAVE_STRDUP 1

/* Define if you have the getrusage function.  */
#define HAVE_GETRUSAGE 1

/* Define if you have the times function.  */
#define HAVE_TIMES 1

/* Define if you have the clock function.  */
#define HAVE_CLOCK 1

/* Define if you have the sleep function.  */
#define HAVE_SLEEP 1

/* Define if you have the <stdlib.h> header file.  */
#define HAVE_STDLIB_H 1

/* Define if you have the <math.h> header file.  */
#define HAVE_MATH_H 1

/* Define if you have the <string.h> header file.  */
#define HAVE_STRING_H 1

/* Define if you have the <strings.h> header file.  */
#define HAVE_STRINGS_H 1

/* Define if you have the <errno.h> header file.  */
#define HAVE_ERRNO_H 1

/* Define if you have the <assert.h> header file.  */
#define HAVE_ASSERT_H 1

/* Define if you can safely include both <sys/time.h> and <time.h>.  */
#define TIME_WITH_SYS_TIME 1

/* Define if you have the <sys/time.h> header file.  */
#define HAVE_SYS_TIME_H 1

/* Define if you have the <time.h> header file.  */
#define HAVE_TIME_H 1

/* Define if you have the <stddef.h> header file.  */
#define HAVE_STDDEF_H 1

/* Define if you have the <unistd.h> header file.  */
#define HAVE_UNISTD_H 1

/* Define if you have the <malloc.h> header file.  */
#define HAVE_MALLOC_H 1

/* Define if you have the <sys/types.h> header file.  */
#define HAVE_SYS_TYPES_H 1

/* Define if you have the <sys/stat.h> header file.  */
#define HAVE_SYS_STAT_H 1

/* Define if you have the <sys/resource.h> header file.  */
#define HAVE_SYS_RESOURCE_H 1

/* Define if you have the <fcntl.h> header file.  */
#define HAVE_FCNTL_H 1

/* Define if you have the <signal.h> header file.  */
#define HAVE_SIGNAL_H 1

/* Define if you have the <sys/socket.h> header file.  */
#define HAVE_SYS_SOCKET_H 1

/* Define if you have the <netdb.h> header file.  */
#define HAVE_NETDB_H 1

/* Define if you have the <netinet/in.h> header file.  */
#define HAVE_NETINET_IN_H 1

/* Define if you have the <sys/param.h> header file.  */
#define HAVE_SYS_PARAM_H 1

/* Define if you have the <sys/times.h> header file.  */
#define HAVE_SYS_TIMES_H 1

/* Define if your compiler supports attribute modifiers  */
/* such as __attribute__ ((unused)) (gcc 2.8.1 does)     */
#define CC_ATTRIBUTE 1

/* Define if your header files use non-Ansi casts for SIG_ERR, SIG_IGN, */
/* or SIG_DFL */
/* #undef CC_BADSIGDEF_CAST */

/* Some machine (o/s) specific problems */

/* Define if unistd.h uses __vfork but does not prototype it */
/* This happens under Irix 6 */
/* #undef CC_PROTO___VFORK */

