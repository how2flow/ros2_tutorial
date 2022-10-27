HEAD
==================
examples package. it is calculator.

- Topic
- Service
- Action
- Parameter
- launch


dependencies
==================
"msg_srv_action_interface_example" in https://github.com/robotpilot/ros2-seminar-examples.git

Topic
==================
- *asynchronous unidirectional message transmission and reception. (1:1), (N:1), (N:N)*

Design
::
  Node (argument)             Node (calculator)
  +-------------------+       +---------------------+
  |                   |       |                     |
  | "Topic Publisher"----------->"Topic Subscriber" |
  |                   |       |                     |
  +-------------------+       +---------------------+

Inst
::
  Topic Publisher
  1. set Node
  2. set QoS
  3. create_publisher<msg type> (+ timer)
  4. define publish function
  ---------------------------
  Topic Subscriber
  1. set Node
  2. set QoS
  3. create_subscription<msg type>
  4. define subscribe function

Total
==================

Design
::
  Node (argument)             Node (calculator)
  +-------------------+       +---------------------+
  |                   |       |                     |
  | "Topic Publisher"----------->"Topic Subscriber" |
  |                   |       |                     |
  +-------------------+       +---------------------+
