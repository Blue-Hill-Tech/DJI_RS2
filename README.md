## Additional Tools and Examples

For users looking for the classic motion-control workflow (recording and replaying
camera movements), please refer to **Log and Replay Movements** — see `README.md`
in the root directory.

For users interested in the ultra-low-latency ZMQ gimbal follower (RS2/RS3 controlled
via real-time Euler angle streaming from a Pico or other device), please check the
folder:rs_pico_zmq_follow/

This folder contains a minimal and high-performance example showing how to stream
orientation data over ZMQ and drive the gimbal at 500 Hz with event-driven control.
