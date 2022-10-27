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

Service
==================
- *It is a synchronous two-way message transmission and reception method (1:1), (N:1)*
- *service-client: service request*
- *service-server: service response*

Design
::
  Node (operator)            Node (calculator)
  +------------------+       +------------------+
  |                  |       |                  |
  | "Service Client"<--------->"Service Server" |
  |                  |       |                  |
  +------------------+       +------------------+

Inst
::
  Service Server
  1. set Node
  2. crete_service<srv type>
  3. define callback function
  ---------------------------
  Service Client
  1. set Node
  2. create_client<srv type>
  3. send_request

Total
==================

Design
::
  Node (argument)              Node (calculator)
  +-------------------+        +--------------------+
  |                   |        |                    |
  | "Topic Publisher"----------->"Topic Subscriber" |
  |                   | (a, b) |                    |
  +-------------------+        |  "Service Server"  |
                               |           ^        |
                               +-----------|--------+
                                           |
  Node (operator)                          | Service-Server (response)
  +------------------+                     |   receive  '+'/'-'/'*'/'/'
  |                  |                     |   and response
  | "Service Client"<----------------------+   what sending result of operate a and b
  |                  |
  +------------------+
     Service-Client (request)
       send ('+'/'-'/'*'/'/') randomly
       and request result of operate a and b
