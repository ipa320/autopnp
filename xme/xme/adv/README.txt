The advanced layer contains all software components that are specific to a
certain application or domain and may not be placed in one of the other
categories. They have the following properties:

1. Advanced components are usually not used in a wide variety of applications,
   but they are rather specific to certain domains or target platforms.

2. Advanced components are not allowed to interact in directly using function
   calls, nor are they allowed to directly access the HAL using function
   calls. The only valid form of communication is via the data centric
   approach.
