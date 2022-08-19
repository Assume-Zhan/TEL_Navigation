# NAV_MEC
---
### File system
- Node Handle : navMec_node.cpp
    - node name : <font color="fx56">"navMec_node"</font>
    - node handle name : <font color="fx56">"navMec_nh"</font>
    - include : navMec_ros.h
- ROS system
    - Refer : <font color="fx56">"navMec_ros.h"</font>
    - Service : srv/navMec_srv.srv
    - Server : "navMec_ser" 
        - Callback function : "serverCB"
    - Client : "navMec_cli"
    - Publisher : "navMec_pub"
    - Subscriber : "navMec_sub"
        - Callback function : "subCB"
- PointController
    - Header file : <font color="fx56">"PointController.h"</font>
    - Library file : <font color="fx56">"PointController.cpp"</font>