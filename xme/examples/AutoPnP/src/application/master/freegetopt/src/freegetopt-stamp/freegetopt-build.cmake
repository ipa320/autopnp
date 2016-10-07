

set(ENV{VS_UNICODE_OUTPUT} "")
set(command "${make}")
execute_process(
  COMMAND ${command}
  RESULT_VARIABLE result
  OUTPUT_FILE "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/freegetopt/src/freegetopt-stamp/freegetopt-build-out.log"
  ERROR_FILE "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/freegetopt/src/freegetopt-stamp/freegetopt-build-err.log"
  )
if(result)
  set(msg "Command failed: ${result}\n")
  foreach(arg IN LISTS command)
    set(msg "${msg} '${arg}'")
  endforeach(arg)
  set(msg "${msg}\nSee also\n  /home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/freegetopt/src/freegetopt-stamp/freegetopt-build-*.log\n")
  message(FATAL_ERROR "${msg}")
else()
  set(msg "freegetopt build command succeeded.  See also /home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/freegetopt/src/freegetopt-stamp/freegetopt-build-*.log\n")
  message(STATUS "${msg}")
endif()
