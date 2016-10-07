The primitive layer contains the *maximum set* of software components with the
following properties:

1. Primitive components are generic enough to be suitable for different types
   of applications (or application domains). If a component is application-
   specific, it must be a member of the "advanced" group instead.

2. In addition to data driven communication, primitive components may directly
   rely on functionality of the HAL by using function calls.

3. Primitive components *may not* directly call other primitive components by
   using function calls. If direct call semantics are required, the component
   must be implemented as an XME core component instead.
