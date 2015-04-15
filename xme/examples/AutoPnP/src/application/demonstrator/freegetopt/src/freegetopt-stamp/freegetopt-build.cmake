

set(ENV{VS_UNICODE_OUTPUT} "")
set(command "${make}")
execute_process(
  COMMAND ${command}
  RESULT_VARIABLE result
  OUTPUT_FILE "/u/rmb/git/autopnp/xme/examples/AutoPnP/src/application/demonstrator/freegetopt/src/freegetopt-stamp/freegetopt-build-out.log"
  ERROR_FILE "/u/rmb/git/autopnp/xme/examples/AutoPnP/src/application/demonstrator/freegetopt/src/freegetopt-stamp/freegetopt-build-err.log"
  )
if(result)
  set(msg "Command failed: ${result}\n")
  foreach(arg IN LISTS command)
    set(msg "${msg} '${arg}'")
  endforeach(arg)
  set(msg "${msg}\nSee also\n  /u/rmb/git/autopnp/xme/examples/AutoPnP/src/application/demonstrator/freegetopt/src/freegetopt-stamp/freegetopt-build-*.log\n")
  message(FATAL_ERROR "${msg}")
else()
  set(msg "freegetopt build command succeeded.  See also /u/rmb/git/autopnp/xme/examples/AutoPnP/src/application/demonstrator/freegetopt/src/freegetopt-stamp/freegetopt-build-*.log\n")
  message(STATUS "${msg}")
endif()
