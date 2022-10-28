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

Action
==================
- It is an asynchronous + synchronous bidirectional message transmission and reception method.
- *action-client: set and send action-goal*
- *action-server: receive action-goal and operate for goal, send action-feedback and action-result*

Design
::
  Node (calculator)             Node (checker)
  +-----------------+           +-----------------+
  |                 |    [1]    |                 |
  |                <------------->                |
  |                 |    [2]    |                 |
  | "Action Server"-------------->"Action Client" |
  |                 |    [3]    |                 |
  |                <------------->                |
  |                 |           |                 |
  +-----------------+           +-----------------+

  [1] Action Goal (Goal Service) "server <-> client"
  [2] Action Feedback (Feedback Topic) "server -> client"
  [3] Action Result (Result Service) "server <-> client"

Inst
::
  Action Server
  1. set Node
  2. create_server<action type>
  3. define callback functions (goal, cancel, accepted)
  ------------------------------
  Action Client
  1. set Node
  2. crete_client<action type>
  3. define callback functions (goal_response, feedback, result)

Total
==================

Design
::
  Node (argument)              Node (calculator)
  +-------------------+        +--------------------+
  |                   |        |                    |          Node (checker)
  | "Topic Publisher"----------->"Topic Subscriber" |          +-----------------+
  |                   | (a, b) |                    |          |                 |
  +-------------------+        |                   <------------>                |
                               |   "Action Server" ------------->"Action Client" |
                               |                   <------------>                |
                               |                    |          |                 |
                               |  "Service Server"  |          +-----------------+
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
