#!/usr/bin/python3

import can
import struct
import threading
import time

from check_sum import *
from ctypes import *


class VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte),
        ("RemoteFlag", c_ubyte),
        ("ExternFlag", c_ubyte),
        ("DataLen", c_ubyte),
        ("Data", c_ubyte * 8),
        ("Reserved", c_ubyte * 3),
    ]


def validate_api_call(data_frame):
    # Validating received frames
    validated = False
    check_sum = ':'.join(data_frame[-4:])
    data = ':'.join(data_frame[:-4])
    if len(data_frame) >= 8:
        if check_sum == calc_crc32(data):
            header = ':'.join(data_frame[:10])
            header_check_sum = ':'.join(data_frame[10:12])
            if header_check_sum == calc_crc16(header):
                validated = True
    return validated


class DJIController:
    def __init__(self, can_bus):
        self.header = 0xAA
        # ENC
        self.enc = 0x00
        # RES
        self.res1 = 0x00
        self.res2 = 0x00
        self.res3 = 0x00
        # SEQ
        self.Seq_Init_Data = 0x0002

        # CAN BUS
        self.bus = can.interface.Bus(bustype="socketcan", channel=can_bus, bitrate=1000000)
        try:
            self.bus.flush_tx_buffer()
        except NotImplementedError:
            pass  # socketcan doesn't support flush_tx_buffer
        self.send_id = int("223", 16)
        self.recv_id = int("222", 16)
        # CAN Specs
        # Data frame len
        self.FRAME_LEN = 8
        self.NORMAL_SEND = 0
        self.SINGLE_SEND = 1
        # Remote Transmission request or not
        self.DATA_FRAME = 0
        self.REMOTE_FRAME = 1
        #
        self.STD_FRAME = 0
        self.EXT_FRAME = 1
        # Data storage
        self.can_recv_msg_buffer = []
        self.can_recv_msg_len_buffer = []
        self.can_recv_buffer_len = 10
        # Standard IDs
        self.cmd_callback_posControl = 0x0E00
        self.cmd_callback_speedControl = 0x0E01
        self.cmd_callback_getGimbalInfo = 0x0E02
        self.cmd_callback_setAngleLimit = 0x0E03
        self.cmd_callback_getAngleLimit = 0x0E04
        self.cmd_callback_setMotorStrength = 0x0E05
        self.cmd_callback_getMotorStrength = 0x0E06
        self.cmd_callback_setPush = 0x0E07
        self.cmd_callback_pushData = 0x0E08
        self.cmd_callback_cameraControl = 0x0D00
        self.cmd_callback_focusControl = 0x0D01
        # Current Position
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.focus = 0
        self.position = [0, 0, 0]
        self.position_time = time.time()
        # Multithreading CAN Callback
        self.request_rate = 0.1  # seconds until next
        self.last_msg_id = 0
        self.debug = False
        # self.f = open("debug.log", "w")
        self.msg_id = 0
        self.b = threading.Thread(name='background', target=self.can_callback)
        self.b.start()
        self.b2 = threading.Thread(name='background', target=self.request_cur_data)
        self.b2.start()

    def request_cur_data(self):
        while True:
            self.get_pos()
            #self.get_foc()
            time.sleep(self.request_rate)

    def seq_num(self):
        # Generate Sequence number by incrementing it
        # global Seq_Init_Data
        if self.Seq_Init_Data >= 0xFFFD:
            self.Seq_Init_Data = 0x0002
        self.Seq_Init_Data += 1
        # Seq_Init_Data = 0x1122
        seq_str = "%04x" % self.Seq_Init_Data
        return seq_str[2:] + ":" + seq_str[0:2]

    def assemble_can_msg(self, cmd_type, cmd_set, cmd_id, data):
        # Assembling a respective CAN message
        if data == "":
            can_frame_data = "{prefix}" + \
                             ":{cmd_set}:{cmd_id}".format(cmd_set=cmd_set, cmd_id=cmd_id)
        else:
            can_frame_data = "{prefix}" + ":{cmd_set}:{cmd_id}:{data}".format(cmd_set=cmd_set, cmd_id=cmd_id, data=data)

        cmd_length = len(can_frame_data.split(":")) + 15
        seqnum = self.seq_num()

        can_frame_header = "{header:02x}".format(header=self.header)  # SOF byte
        can_frame_header += ":" + ("%04x" % (cmd_length))[2:4]  # 1st length byte
        can_frame_header += ":" + ("%04x" % (cmd_length))[0:2]  # 2nd length byte
        can_frame_header += ":" + \
                            "{cmd_type}".format(cmd_type=cmd_type)  # Command Type
        can_frame_header += ":" + "{enc:02x}".format(enc=self.enc)  # Encoding
        can_frame_header += ":" + "{res1:02x}".format(res1=self.res1)  # Reserved 1
        can_frame_header += ":" + "{res2:02x}".format(res2=self.res2)  # Reserved 2
        can_frame_header += ":" + "{res3:02x}".format(res3=self.res3)  # Reserved 3
        can_frame_header += ":" + seqnum  # Sequence number
        can_frame_header += ":" + calc_crc16(can_frame_header)
        # hex_seq = [eval("0x" + hex_num) for hex_num in can_frame_header.split(":")]
        whole_can_frame = can_frame_data.format(prefix=can_frame_header)
        whole_can_frame += ":" + calc_crc32(whole_can_frame)
        whole_can_frame = whole_can_frame.upper()
        #
        # print("Header: ", can_frame_header)
        # print("Total: ", whole_can_frame)
        return whole_can_frame

    def set_ypr(self, ypr, t):
        self.set_pos(ypr[0], ypr[2], ypr[1], time_for_action=int(t * 10.0))


    def set_pos(self, yaw, roll, pitch, ctrl_byte=0x01, time_for_action=20):  # 20 = 2 sec
        # Set Gimbal Position
        yaw = int(yaw * 10.0)
        roll = int(roll * 10.0)
        pitch = int(pitch * 10.0)

        # Turn Position Data into CAN message
        # yaw, roll, pitch in 0.1° steps (-1800,1800)
        # ctrl_byte always to 1
        # time_for_action to define speed in 0.1sec
        hex_data = struct.pack('<3h2B', yaw, roll, pitch,
                               ctrl_byte, time_for_action)

        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                                    cmd_id='00', data=cmd_data)
        self.send_cmd(cmd)

    def get_pos(self):
        hex_data = struct.pack('<1B', 0x01)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                                    cmd_id='02', data=cmd_data)
        self.send_cmd(cmd)

    def set_foc(self, position, cmd_sub_id=0x01, ctl_type=0x00, data_length=0x02):
        # 0-4096 absolute position values
        hex_data = struct.pack('<3B1H', cmd_sub_id, ctl_type, data_length, position)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                                    cmd_id='12', data=cmd_data)
        self.send_cmd(cmd)

    def get_foc(self):
        hex_data = struct.pack('<2B', 0x15, 0x00)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                                    cmd_id='12', data=cmd_data)
        self.send_cmd(cmd)

    def set_speed(self, yaw, roll, pitch, ctrl_byte=0x80):
        yaw = int(yaw * 10.0)
        roll = int(roll * 10.0)
        pitch = int(pitch * 10.0)
        hex_data = struct.pack('<3hB', yaw, roll, pitch, ctrl_byte)
        pack_data = ['{:02X}'.format(i) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                                    cmd_id='01', data=cmd_data)
        # print('cmd---data {}'.format(cmd))
        self.send_cmd(cmd)

    def enable_hand_push(self):
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E', cmd_id='07', data='01')
        self.send_cmd(cmd)

    def send_cmd(self, cmd):
        # Preparing data for bus output
        data = [int(i, 16) for i in cmd.split(":")]
        # self.lastSendData = data
        # status  =False
        self.send_data(self.send_id, data)
        # send_data(send_id, cmd)

    def send_data(self, can_id, data):
        # Pushing Data out the CAN bus
        data_len = len(data)
        full_frame_num, left_len = divmod(data_len, self.FRAME_LEN)
        if left_len == 0:
            frame_num = full_frame_num
        else:
            frame_num = full_frame_num + 1
        send_buf = (VCI_CAN_OBJ * (frame_num))()
        data_offset = 0
        for i in range(full_frame_num):
            send_buf[i].ID = can_id
            send_buf[i].SendType = self.NORMAL_SEND
            send_buf[i].RemoteFlag = self.DATA_FRAME
            send_buf[i].ExternFlag = self.STD_FRAME
            send_buf[i].DataLen = self.FRAME_LEN
            for j in range(self.FRAME_LEN):
                send_buf[i].Data[j] = data[data_offset + j]
            data_offset += self.FRAME_LEN

            # If there is data left over, the last frame isn't 8byte long
            if left_len > 0:
                send_buf[frame_num - 1].ID = can_id
                send_buf[frame_num - 1].SendType = self.NORMAL_SEND
                send_buf[frame_num - 1].RemoteFlag = self.DATA_FRAME
                send_buf[frame_num - 1].ExternFlag = self.STD_FRAME
                send_buf[frame_num - 1].DataLen = left_len
                for j in range(left_len):
                    send_buf[frame_num - 1].Data[j] = data[data_offset + j]

            for i in range(frame_num):
                frame = bytearray()
                for j in range(send_buf[i].DataLen):
                    frame.append(send_buf[i].Data[j])
                msg = can.Message(arbitration_id=0x223, data=frame, is_extended_id=False)
                try:
                    self.bus.send(msg)
                    # print("Message sent on {}".format(bus.channel_info))
                except can.CanError:
                    print("Message NOT sent")

    def can_buffer_to_full_frame(self):
        # assembling individual CAN messages into whole FRAMES
        full_msg_frames = []
        full_frame_counter = 0
        for i in range(len(self.can_recv_msg_buffer)):
            msg = self.can_recv_msg_buffer[i]
            length = self.can_recv_msg_len_buffer[i]
            msg = msg[:length]
            cmd_data = ':'.join(msg)
            # print("len: " + str(length) + " - " +
            #       str(msg) + " -> " + cmd_data)
            if msg[0] == "AA":
                full_msg_frames.append(msg)
                full_frame_counter += 1
            if msg[0] != "AA" and (full_frame_counter > 0):
                # full_msg_frames[-1] += ":"
                for byte in msg:
                    full_msg_frames[-1].append(byte)
        return full_msg_frames

    def parse_focus_motor_response(self, data_frame):
        pos_data = data_frame[-8:-4]
        value = int('0x' + pos_data[3] + pos_data[2] + pos_data[1] + pos_data[0], base=16)
        self.focus = value

    def parse_position_response(self, data_frame):
        pos_data = data_frame[16:-4]
        if len(pos_data) < 6:
            return  # Invalid data length, skip parsing
        yaw = int('0x' + pos_data[1] + pos_data[0], base=16)
        roll = int('0x' + pos_data[3] + pos_data[2], base=16)
        pitch = int('0x' + pos_data[5] + pos_data[4], base=16)
        if yaw > 1800:
            yaw -= 65538
        if roll > 1800:
            roll -= 65538
        if pitch > 1800:
            pitch -= 65538
        self.yaw = yaw * 0.1  # * np.pi / 180
        self.roll = roll * 0.1  # * np.pi / 180
        self.pitch = pitch * 0.1  # * np.pi / 180
        self.position = [self.yaw, self.pitch, self.roll]
        self.position_time = time.time()

    def can_callback(self):
        while True:
            if 65536 < self.last_msg_id:
                self.last_msg_id = 0
            for msg in self.bus:
                if msg.arbitration_id == self.recv_id:
                    str_data = ['{:02X}'.format(struct.unpack('<1B', i.to_bytes(1, 'big'))[
                                                    0]) for i in msg.data]
                    self.can_recv_msg_buffer.append(str_data)
                    self.can_recv_msg_len_buffer.append(msg.dlc)
                    if len(self.can_recv_msg_buffer) > self.can_recv_buffer_len:
                        self.can_recv_msg_buffer.pop(0)
                        self.can_recv_msg_len_buffer.pop(0)
                    full_msg_frames = self.can_buffer_to_full_frame()
                    for hex_data in full_msg_frames:
                        if validate_api_call(hex_data):
                            cur_msg_id = int('0x' + hex_data[9] + hex_data[8], base=16)
                            if True:
                                self.last_msg_id = cur_msg_id
                                request_data = ":".join(hex_data[12:14])
                                if request_data == "0E:02":
                                    self.parse_position_response(hex_data)
                                elif request_data == "0E:12":
                                    self.parse_focus_motor_response(hex_data)
                        else:
                            # invalid message
                            pass
