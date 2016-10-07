The core layer contains the *minimum set* of software components with the
following properties:

1. Core components are absolutely necessary in (almost) every application
   (however not all components will always be present in all applications).

2. In addition to data centric communication, core components may directly
   rely on functionality of the HAL by using function calls.

3. Core components may directly call other core components by using function
   calls.
