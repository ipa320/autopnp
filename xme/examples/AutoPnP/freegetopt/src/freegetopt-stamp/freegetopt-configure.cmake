

set(ENV{VS_UNICODE_OUTPUT} "")
set(command "/usr/bin/cmake;-DCMAKE_C_FLAGS:STRING=-D_GNU_SOURCE -std=c99 -W -Wextra -Wall -Wbad-function-cast -Wcomments -Wunused-macros -Wendif-labels -Wmissing-declarations -Wmissing-prototypes -Wnested-externs -Wold-style-definition -Wstrict-prototypes -Wdeclaration-after-statement -Werror -Wno-missing-field-initializers -fPIC -fstrict-overflow -m64 -m64 -m64 -m64 -D_CRT_SECURE_NO_WARNINGS;-DCMAKE_CXX_FLAGS:STRING=-D_GNU_SOURCE -ansi -W -Wextra -Wall -Wshadow -Wunused-variable -Wunused-parameter -Wunused-function -Wunused -Wno-system-headers -Wno-deprecated -Woverloaded-virtual -Wwrite-strings -Wno-missing-field-initializers -fPIC -fstrict-overflow -m64 -m64 -m64 -m64 -D_CRT_SECURE_NO_WARNINGS;-GUnix Makefiles;/home/josh/xme/external/freegetopt")
execute_process(
  COMMAND ${command}
  RESULT_VARIABLE result
  OUTPUT_FILE "/home/josh/xme/examples/AutoPnP/freegetopt/src/freegetopt-stamp/freegetopt-configure-out.log"
  ERROR_FILE "/home/josh/xme/examples/AutoPnP/freegetopt/src/freegetopt-stamp/freegetopt-configure-err.log"
  )
if(result)
  set(msg "Command failed: ${result}\n")
  foreach(arg IN LISTS command)
    set(msg "${msg} '${arg}'")
  endforeach(arg)
  set(msg "${msg}\nSee also\n  /home/josh/xme/examples/AutoPnP/freegetopt/src/freegetopt-stamp/freegetopt-configure-*.log\n")
  message(FATAL_ERROR "${msg}")
else()
  set(msg "freegetopt configure command succeeded.  See also /home/josh/xme/examples/AutoPnP/freegetopt/src/freegetopt-stamp/freegetopt-configure-*.log\n")
  message(STATUS "${msg}")
endif()
