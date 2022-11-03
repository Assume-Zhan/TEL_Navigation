# NAV_MEC
> Navigation node

---

## File system
- Node Handle : Node Handle
    - Node name : navMec_node
    - NH name : navMec_nh
    - header file : navMec_ros.h
- Point Controller
    - header file : PointController.h
    - library file : PointController.cpp
- Pure pursuit
    - header file : PurePursuit.h
    - library file : PurePursuit.cpp

---

## Launch file 

---

## Usage

---

## :warning: Important note
- SLOWDOWN should be design properly
- Use private parameters
```cpp=1
ros::param::get("~debugmode", debug_mode);
```
- If the node is not the main machine, debug_mode should be set by bool value.
    - If we set the debug mode as define, it is diffcult to change mode by launch file