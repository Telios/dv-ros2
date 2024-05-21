# DV ROS2 Accumulation

This is a port of the original [DV ROS Accumulation project](https://gitlab.com/inivation/dv/dv-ros/-/tree/master/dv_ros_accumulation). It provides two nodes: accumulation and edge_map nodes. These nodes can be used to accumulate
a stream of events into a stream of images representing the events. Stream of events is sliced into chunks by amount or
time depending on the configuration and is accumulated into a frame representation.