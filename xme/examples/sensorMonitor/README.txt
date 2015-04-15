========================================
  CHROMOSOME Example "sensorMonitor"
========================================

This example demonstrates a number of CHROMOSOME features in a concentrated
way. It consists of multiple multiple nodes in an XME ecosystem that interact
with each other. The demonstrated features are:
 - Data centric communication between source and sink components (based on
   topics and attributes), both local (on one execution platform) and remote
   (distributed execution platform in same subnet).
 - Plugging of new components at runtime and subsequent reconfiguration of
   the communication schema.
 - Login of previously unknown XME nodes at runtime, announcement of offered
   services and subsequent reconfiguration of the communication schema.

Scenario:
The XME ecosystem under consideration may contain an arbitrary number of
Sensor and Monitor components. A Sensor component publishes a sensor value
that can be displayed by one or more Monitor components. Depending on initial
configuration and plug & play at runtime, the information being displayed
changes over time.

Application-specific topics:
 1. Sensor Value
    This topic represents a value emitted by a Sensor component that can be
    displayed by a Monitor component.

Application-specific components:
 1. Monitor
    The Monitor component subscribes to topics of type Sensor Value (possibly
    with a restriction with respect to attributes) and displays the received
    values as soon as they are received.
 2. Sensor
    The Sensor component emits a value of topic Sensor Value in every cycle it
    is executed. The value being emitted is the amount of free space on a
    hard drive partition in the system under consideration. The partition to
    monitor is configured at startup of the component.

Application nodes:
 1. monitorNode
    Contains Plug and Play Manager, Login Manager and a Monitor component.
    Required for plug & play and login capabilities of other nodes.
 2. sensorNode
    Contains just a Sensor component.
 3. emptyNode
    Node with initially no running components, but with the capability to
    add Sensor and/or Monitor components at runtime using plug & play
    mechanisms.
 4. lonelyNode
    Node with initially no running components, but with the capability to
    add Sensor and/or Monitor components at runtime using plug & play
    mechanisms. In addition, this node is not specified in the model where
    monitorNode, sensorNode and emptyNode are specified, hence the integration
    of this node into the XME ecosystem requires login mechanisms.

Used CHROMOSOME services:
 - Platform Abstraction (HAL): Required as a base for implementation of
   higher-level services.
 - Time-triggered execution (Execution Manager).
 - Data handling (Data Handler, Broker).
 - Communication (Marshaler, Demarshaler, UDP Send, UDP Receive).
 - Plug and play: Required for dynamic instantiation of a sensor or monitor
   component on emptyNode and for activation of software components after login
   of lonelyNode (Plug and Play Manager, Plug and Play Client).
 - Login: Required for dynamic integration of lonelyNode into the XME
   ecosystem (Login Manager, Login Client).

Platform support:
This example has been tested and is currently maintained on Linux and Windows
target systems.
