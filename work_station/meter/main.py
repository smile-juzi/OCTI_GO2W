import cv2
from rknnpool import rknnPoolExecutor
from func import myFunc  # 你需要定义的自定义函数
from read_meter import MeterReader
from dect_meter import capture_meter_frame
import numpy as np
import socket
import struct
import threading
import time
import video_number

INSTRUCTION_REQUEST = 1
INSTRUCTION_ALIVE = 2
INSTRUCTION_ACK = 3
INSTRUCTION_FINAL = 4
INSTRUCTION_ROBOTCONTROL = 5

#第一位
REQUEST_DATA = 0
GIVE_DATA = 1
CONTROL_COMMOND = 2

#第二位与第三位
SERVER = 0
VISION = 1
ROBOT = 2
METER = 3

#第四位
SUB_FACE = 0
SUB_METER = 1
SUB_SMOKE_HAT = 2
SUB_INF = 3  #红外入侵检测
SUB_THR = 4  #温度检测
SUB_GAS = 5  #气体检测

#第五位
EXIST = 0
NOT_EXIST = 1
FINSH = 3 #无关

#仪表检测数据帧第六位:数据类型  人脸第六位为人脸信息
METER_TYPE_XIAOFANG = 0
METER_TYPE_AIR = 1
METER_TYPE_JIXIE = 2

#抽烟安全帽数据帧第六位
HAT = 0
NO_HAT = 1
SMOKE = 2
NOT_SMOKE = 3


#仪表检测控制帧第六位
CLOSE_ALL = 0
START = 1
CLOSE = 2
#仪表检测数据帧第八位  温度检测第八位为温度
#读数

#第七和第九为空出


frame_format = 'QQidd'
frame_format_meter = 'IIIIIIIff'

RANGE_MAP = {
    "消防水压表": 1.0,
    "空气房气压表": 1.6,
    "机械气压表": 1.0
}

meter_read_dict = {
                    0: 0.0,
                    1: 0.0,
                    2: 0.0
                   }
# 初始化模型池
def initialize_model_pool(model_path, tpes=6):
    pool = rknnPoolExecutor(
        rknnModel=model_path,
        TPEs=tpes,
        func=myFunc
    )
    return pool

def open_camera(camera_numbers):
    for number in camera_numbers:
        cap = cv2.VideoCapture(number)
        if cap.isOpened():
            print(f"Found openable camera: {number}")
            return cap
    return None


def send_frame_to_client(sock, client_address, max_retries=1, timeout=1):
    sock.settimeout(timeout)

    # 发送UDP消息到客户端
    data = (2, 3, 1, 1, 0, 0, 0, 0, 0)
    packed_data = struct.pack(frame_format_meter, *data)

    for attempt in range(max_retries):
        try:

            # 发送打包后的数据到客户端
            sock.sendto(packed_data, client_address)
            print("已发送")
            # 接收来自客户端的回应
            rec, server = sock.recvfrom(1024)  # 等待最大1024字节的响应
            unpacked_data = struct.unpack(frame_format_meter, rec)
            print(f"Received response from {server}: {unpacked_data}")
            if unpacked_data[4] == 3:
                return True

        except socket.timeout:
            print("Error: No response from client (timeout).")

    return False

def send_last_to_client(sock, client_address):
    # 发送UDP消息到客户端
    data = (2, 3, 1, 1, 0, 1, 0, 0, 0)
    packed_data = struct.pack(frame_format_meter, *data)
    # 发送打包后的数据到客户端
    sock.sendto(packed_data, client_address)
    print("已发送")

class UDPClient:
    def __init__(self, server_ip, server_port):
        client_ip = "127.0.0.1"
        client_port = 8889

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.client_socket.bind((client_ip, client_port))
        self.server_address = (server_ip, server_port)
        self.is_detection_complete = False
        self.is_detection_start = False
        self.seq = 0
        self.cap = 0
        self.lock = threading.Lock()  # 用于保护共享变量

    def create_reply_frame(self, frame_type, seq, instruction, reserver_data_1, reserver_data_2):
        return struct.pack(frame_format, frame_type, seq, instruction, reserver_data_1, reserver_data_2)

    def send_frame(self, frame):
        self.client_socket.sendto(frame, self.server_address)

    def listen_for_commands(self):
        global meter_read_dict
        while True:  # 保持监听
            try:
                frame_data, address = self.client_socket.recvfrom(1024)

                server_add = ("127.0.0.1", 8111)
                # 服务端请求数据
                if address == server_add:
                    print("receive server frame")
                    frame_data = struct.unpack(frame_format_meter, frame_data)

                    if frame_data[0] == 0 and frame_data[2] == 3:
                        print("server data")
                        # 第一个表读数
                        print(f"meter: {meter_read_dict[0]},{meter_read_dict[1]},{meter_read_dict[2]}")
                        data_meter = (1, 3, 0, 1, 1, 0, 0, meter_read_dict[0], 0)
                        pack_data = struct.pack(frame_format_meter, *data_meter)
                        self.client_socket.sendto(pack_data, server_add)

                        # 第二个表读数
                        data_meter = (1, 3, 0, 1, 1, 1, 0, meter_read_dict[1], 0)
                        pack_data = struct.pack(frame_format_meter, *data_meter)
                        self.client_socket.sendto(pack_data, server_add)

                        # 第三个表读数
                        data_meter = (1, 3, 0, 1, 1, 2, 0, meter_read_dict[2], 0)
                        pack_data = struct.pack(frame_format_meter, *data_meter)
                        self.client_socket.sendto(pack_data, server_add)

                #运动控制端控制检测仪表
                else:
                    parsed_frame = self.parse_frame(frame_data)
                    print(f'收到命令：{parsed_frame}')
                    instruction = parsed_frame['instruction']

                    if instruction == INSTRUCTION_REQUEST:
                        print("收到命令REQUEST命令：开始检测")
                        if self.is_detection_start is False:
                            with self.lock:
                                self.is_detection_start = True
                                self.is_detection_complete = False
                            self.send_ack(parsed_frame['seq'])
                            self.start_meter_detection()
                        else:
                            self.send_ack(parsed_frame['seq'])

                    elif instruction == INSTRUCTION_ALIVE:
                        #print("收到命令ALIVE命令：保持连接")
                        if self.is_detection_start is False:
                            self.send_final()
                        if self.is_detection_complete is True:
                            self.send_final()
                            print("alive final")
                        else:
                            self.send_ack(parsed_frame['seq'])
                    elif instruction == INSTRUCTION_FINAL:
                        print('检测完成')
                        with self.lock:
                            self.is_detection_complete = True
                            self.is_detection_start = False

            except Exception as e:
                print(f"命令接收错误：{e}")

    def send_ack(self, seq):
        ack_frame = self.create_reply_frame(2, seq, INSTRUCTION_ACK, 0, 0)
        self.send_frame(ack_frame)

    def send_final(self):
        final_frame = self.create_reply_frame(2, self.seq, INSTRUCTION_FINAL, 0, 0)
        self.send_frame(final_frame)

    def parse_frame(self, frame_data):
        frame = struct.unpack(frame_format, frame_data)
        return {
            'frameType': frame[0],
            'seq': frame[1],
            'instruction': frame[2],
            'reserveData_1': frame[3],
            'reserveData_2': frame[4]
        }

    def start_meter_detection(self):
        detection_thread = threading.Thread(target=self.meter_detection_process)
        detection_thread.start()

    def meter_detection_process(self, max_retries=3):
        global meter_read_dict
        retries = 0
        modelPath = "./rknnModel/yolov8_seg_newer.rknn"
        tpes = 6
        pool = initialize_model_pool(modelPath, tpes)

        # 创建 UDP 套接字
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #sock.settimeout(5)
        client_address = ('127.0.0.1', 8870)  # 客户端 IP 和端口

        # Step 1: 发送请求帧到客户端，并等待回应
        print("发送帧")

        response = send_frame_to_client(sock, client_address)
        if response is not True:
            print("Error: Client response not OK. Aborting camera operation.")
            self.send_final()
            print("send final")
            send_last_to_client(sock, client_address)
            return

        cap = open_camera(video_number.rgb_numbers)

        while retries < max_retries:
            image, category = capture_meter_frame(cap)

            if image is None or not category:
                print(f"Error: Could not capture image. Retrying {retries + 1}/{max_retries}")
                retries += 1
                continue

            range_value = RANGE_MAP.get(category[0], 1)
            # 将图像放入处理池
            pool.put(image)
            result, flag = pool.get()

            if flag:
                result_img, pointer_mask, scale_mask = result
                if pointer_mask is None or scale_mask is None:
                    print("Error: Mask processing failed.")
                    retries += 1
                    continue

                pointer_mask = np.clip(np.sum(pointer_mask, axis=0) * 255, 0, 255).astype(np.uint8)
                scale_mask = np.clip(np.sum(scale_mask, axis=0) * 255, 0, 255).astype(np.uint8)

                pointer_mask = cv2.resize(pointer_mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_LINEAR)
                scale_mask = cv2.resize(scale_mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_LINEAR)

                # 使用 MeterReader 计算读数
                meter_reader = MeterReader(image)
                result = meter_reader(pointer_mask, scale_mask)
                if result is None:
                    print(f"Error: Failed to process image. Retrying {retries + 1}/{max_retries}")
                    retries += 1
                    continue

                if result < 0.01 or result > 1:
                    result = 0
                # 将结果乘以量程值
                else:
                    result = result * range_value

                    if category[0] == "消防水压表":
                        meter_read_dict[0] = result
                        print('xiaofang')
                    elif category[0] == "空气房气压表":
                        meter_read_dict[1] = result
                        print('kongqi')
                    elif category[0] == "机械气压表":
                        meter_read_dict[2] = result
                        print('jixie')

                print(f"result =  {result}")
                with self.lock:
                    self.is_detection_complete = True
                    self.is_detection_start = False
                self.send_final()
                print("send final")

                break
            else:
                print(f"Error: Failed to process image. Retrying {retries + 1}/{max_retries}")
                retries += 1


        with self.lock:
            self.is_detection_complete = True
            self.is_detection_start = False
        self.send_final()
        print("send final")
        cap.release()
        print("cap release")
        send_last_to_client(sock, client_address)
        sock.close()
        pool.release()
        cv2.destroyAllWindows()


def main():
    server_ip = "127.0.0.1"
    server_port = 6007

    client = UDPClient(server_ip, server_port)

    command_thread = threading.Thread(target=client.listen_for_commands)
    command_thread.start()

    while True:  # 持续运行，直到手动终止
        with client.lock:
            if client.is_detection_complete:
                print('检测完成，等待新命令')

        time.sleep(1)


if __name__ == '__main__':
    main()

# # 模型推理并计算仪表读数
#
# def process_image_and_read_meter(image, pool, range_value):
#     # 将图像放入处理池
#     pool.put(image)
#     result, flag = pool.get()
#
#     if flag:
#
#         result_img, pointer_mask, scale_mask = result
#         if (pointer_mask is None) | (scale_mask is None):
#             return None, False
#         pointer_mask = np.sum(pointer_mask, axis=0)
#         pointer_mask = pointer_mask * 255
#         scale_mask = np.sum(scale_mask, axis=0)
#         scale_mask = scale_mask * 255
#         pointer_mask = pointer_mask.astype(np.uint8)
#         scale_mask = scale_mask.astype(np.uint8)
#         pointer_mask = cv2.resize(pointer_mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_LINEAR)
#         scale_mask = cv2.resize(scale_mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_LINEAR)
#
#
#         # 使用 MeterReader 计算读数
#         meter_reader = MeterReader(image)
#         if meter_reader is None:
#             return None, False
#
#         result = meter_reader(pointer_mask, scale_mask)
#         if result is None:
#             return None, False
#
#         # 将结果乘以量程值
#         result = result * range_value
#         return result, True
#     else:
#         return None, False
#
#
#
#
# # 主逻辑：尝试从摄像头获取图像并进行模型推理，最多重试3次
# def detect_meter_with_retries(cap, pool, max_retries=3):
#     retries = 0
#     while retries < max_retries:
#         image, category = capture_meter_frame(cap)
#
#         if image is None or not category:
#             print(f"Error: Could not capture image. Retrying {retries + 1}/{max_retries}")
#             retries += 1
#             continue
#
#         range_value = RANGE_MAP.get(category[0], 1)
#         result, success = process_image_and_read_meter(image, pool, range_value)
#
#         if success:
#             return result, True
#         else:
#             print(f"Error: Failed to process image. Retrying {retries + 1}/{max_retries}")
#             retries += 1
#
#     # 如果三次重试都失败，则返回错误标志位
#     return None, False
#
#
# # 主程序入口
# if __name__ == "__main__":
#     # 模型路径
#     modelPath = "./rknnModel/yolov8_seg_new.rknn"
#
#     # 初始化摄像头
#     cap = cv2.VideoCapture(0)
#
#     # 初始化模型池
#     pool = initialize_model_pool(modelPath)
#
#     # 尝试进行仪表检测与推理
#     result, success = detect_meter_with_retries(cap, pool)
#
#     if success:
#         print(f"Meter reading: {result}")
#     else:
#         print("Error: Meter reading failed after multiple attempts.")
#
#     # 释放资源
#     cap.release()
#     pool.release()
#     cv2.destroyAllWindows()
