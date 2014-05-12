========================================
  CHROMOSOME Example "calculator"
========================================

This example demonstrates the client/server based communication scheme in
CHROMOSOME, which we call request/response (RR). An RR communication pattern
consists of two topics: a request topic and a response topic. A data packet
of type request is sent from the client to the server "on demand" and contains
all information necessary for the server to process the request. Subsequently,
the server shall reply with a data packet of type response, which contains the
server's answer.
The example shown here is a "calculator service" that allows to answer
simple calculation queries. The request topic consists of two operands and
one of the operations addition, subtraction, multiplication and division.
When a request is sent to a calculator server, the server shall reply with
the result of the requested operation.

Scenario:
The example contains a calculator server node and two "clients" that send
random requests at regular intervals. Both clients and servers display their
current status (sent requests, processed requests, received responses) on
their standard output.

Application-specific topics:
 1. Calculator Request
    This topic contains two integer numbers that serve as operators and one
    enumeration which can be set to one of the four basic arithmetic operations
    addition, subtraction, multiplication and division.
 2. Calculator Response
    This topic contains a double-precision floating-point number that indicates
    the result of the operation. Notice that in this simple example, the
    response does not indicate what was the respective request.

Application-specific components:
 1. Calculator
    The Calculator component waits for client requests and handles them as they
    arrive. Furthermore, it provides respective output on the console.
 2. Client
    A Client component sends a request using random numbers and operations
    and displays both request and response in its console.

Application nodes:
 1. serverNode
    The serverNode hosts one Calculator component that is executed every 200ms.
 2. clientNode1
    The clientNode hosts one Client component that is executed every 1000ms.
 3. clientNode2
    The clientNode hosts another Client component that is executed every 1000ms.

Used CHROMOSOME services:
 - Platform Abstraction (HAL): Required as a base for implementation of
   higher-level services.
 - Time-triggered execution (Execution Manager).
 - Data handling (Data Handler, Broker).
 - Communication (Channel Injector, Channel Selector, Marshaler, Demarshaler,
   UDP Send, UDP Receive).

Platform support:
This example has been tested and is currently maintained on Linux and Windows
target systems.
