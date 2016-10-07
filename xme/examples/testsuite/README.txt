========================================
  CHROMOSOME Example "testsuite"
========================================

This example's only function is to serve as a configuration to run all
CHROMOSOME-related unit tests. For this purpose, the example's main
CMakeLists.txt file enables unit testing and creates a "fake" executable
target that depends on all existing components that offer unit tests.
Configuring this project can hence be used to create a Makefile or Visual
Studio solution that allows to easily create, modify and run unit tests
while adding or extending existing components.

Scenario:
This scenario consists of just a single node that has no meaningful
functionality. It rather generates an executable target for every unit
test that can be triggered independently to verify whether the respective
components pass the tests.

Application-specific topics:
 (none)

Application-specific components:
 (none)

Application nodes:
 1. client
    "Fake" node for specification of dependencies against all unit tests.

Used CHROMOSOME services:
All services that are unit-tested, among them:
 - Platform Abstraction (HAL).
 - Time-triggered execution (Execution Manager).
 - Data handling (Data Handler, Broker).
 - Communication (Marshaler, Demarshaler, UDP Send, UDP Receive).
 - Plug and play (Login Manager, Login Client).

NOTE: This list is not exhaustive. Please inspect the output from a CMake
      configuration run of this example in order to see a complete list of the
      components being tested.

Platform support:
This example has been tested and is currently maintained on Linux and Windows
target systems.
