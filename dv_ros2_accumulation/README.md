# DV ROS2 Accumulation

This is a port of the original [DV ROS Accumulation project](https://gitlab.com/inivation/dv/dv-ros/-/tree/master/dv_ros_accumulation). It provides two nodes: accumulation and edge_map nodes. These nodes can be used to accumulate
a stream of events into a stream of images representing the events. Stream of events is sliced into chunks by amount or
time depending on the configuration and is accumulated into a frame representation.

## Modes
Depending on the mode, the accumulation node can accumulate events in different ways. The modes are:
- `FRAME`: event accumulation to reconstruct an image.
- `EDGE`: event accumulation to reconstruct an edge map.

The edge map representation can be
favorable for some computer vision applications. The edge map uses a simplified and optimized accumulation approach
for the edge extraction that is more efficient than regular accumulation.

See `config/config.yaml` for more details.