A basic bot
===========

A basic bot can walk in a couple of predefined gaits.

Gaits can use simple animation technique similar to how XYZ robot used
to work

Code can be just a set of custom python scripts

A better bot
============

A better robot would use an Inverse Kinematics to handle its gaits. It’s
able to adjust body height and have at least two different gaits
controlled via IK solver.

Implementation can still use custom python scripts.

A good bot
==========

A good bot uses ROS2 to handle its work.

Servos are controlled by a controller node. It receives commands from
joystick via standard ROS messages.

It publishes transformations that can be observed in rvis

A great bot
===========

A great bot uses Move It 2 for its IK solvers to avoid any collisions,
has self balance and feet sensors to cover uneven terrrain.

A dream bot
===========

A fully autonomous bot that is controlled by AI. It can walk on any
terrain. See people around it with its eye camera and interact with them
via voice. It can carry little objects on its back, requiring assistance
to load them in.

It can be modified with the BB gun turret and sensor plate to
participate in the Mech Warfare competition.
