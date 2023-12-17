# Monitor ROS2 topics in foxglove studio

In Moonraker platform, we use foxglove studio to monitor published topics. \
To monitor published topics, you need to make sure that 
- Host PC is on the same network as robot PC
- You have foxglove studio in your host PC


## Setup

### Robot setup
```
# Since I have already registered alias, just run the following.
start_foxglove_bridge
```

### Host setup

1. Open "Open connection > Foxglove WebSocket"
2. Type `ws://{ip address of robot}:8765`