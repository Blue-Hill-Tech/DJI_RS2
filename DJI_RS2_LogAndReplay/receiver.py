# receiver.py - 在 Linux 上运行
import zmq

context = zmq.Context()
socket = context.socket(zmq.PULL)

socket.bind("tcp://0.0.0.0:5555")
print("[Linux] 等待接收数据...")

while True:
    msg = socket.recv_string()
    print("[Linux] 收到:", msg)