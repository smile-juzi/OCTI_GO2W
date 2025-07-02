import cv2
import threading
import queue
from rknnpool import rknnPoolExecutor
from func import myFunc
from read_meter import MeterReader
from dect_meter import capture_meter_frame
import numpy as np

# 定义量程映射
RANGE_MAP = {
    "消防水压表": 1.0,
    "空气房气压表": 1.6,
    "机械气压表": 1.0
}

# 创建一个队列用于线程间通信
command_queue = queue.Queue()

# 摄像头检测与识别工作线程
def meter_detection_task(command_queue, result_queue):
    cap = cv2.VideoCapture(0)

    while True:
        command = command_queue.get()  # 获取主线程的命令
        if command == "start_detection":
            print("启动仪表检测...")

            # 执行仪表检测逻辑
            image, category = capture_meter_frame(cap)
            range_value = RANGE_MAP.get(category, 1)

            if image is None:
                print("Error: Could not read the image.")
                result_queue.put("Error: Could not read the image.")
                continue

            # 模型路径
            modelPath = "./rknnModel/yolov8_seg.rknn"
            # 线程数, 增大可提高处理速度
            TPEs = 6

            # 初始化rknn池
            pool = rknnPoolExecutor(
                rknnModel=modelPath,
                TPEs=TPEs,
                func=myFunc)

            # 将图片放入RKNN池进行检测
            pool.put(image)

            # 获取处理结果
            result, flag = pool.get()

            if flag:
                result_img, pointer_mask, scale_mask = result

                # 处理结果
                pointer_mask_combined = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
                scale_mask_combined = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)

                # 处理 scale_mask
                for i in range(scale_mask.shape[0]):
                    scale_mask_current = scale_mask[i]
                    if scale_mask_current.dtype == np.bool_:
                        scale_mask_current = scale_mask_current.astype(np.uint8) * 255
                    scale_mask_resized = cv2.resize(scale_mask_current, (image.shape[1], image.shape[0]),
                                                    interpolation=cv2.INTER_NEAREST)
                    scale_mask_combined = cv2.bitwise_or(scale_mask_combined, scale_mask_resized)

                # 处理 pointer_mask
                for i in range(pointer_mask.shape[0]):
                    pointer_mask_current = pointer_mask[i]
                    if pointer_mask_current.dtype == np.bool_:
                        pointer_mask_current = pointer_mask_current.astype(np.uint8) * 255
                    pointer_mask_resized = cv2.resize(pointer_mask_current, (image.shape[1], image.shape[0]),
                                                      interpolation=cv2.INTER_NEAREST)
                    pointer_mask_combined = cv2.bitwise_or(pointer_mask_combined, pointer_mask_resized)

                point_mask_resized = cv2.resize(pointer_mask_combined, (image.shape[1], image.shape[0]),
                                                interpolation=cv2.INTER_NEAREST)
                dail_mask_resized = cv2.resize(scale_mask_combined, (image.shape[1], image.shape[0]),
                                               interpolation=cv2.INTER_NEAREST)

                # 使用 MeterReader 进行读数计算
                meter_reader = MeterReader(image)
                result = meter_reader(point_mask_resized, dail_mask_resized)
                result = result * range_value
                print(f"Meter reading: {result}")
                result_queue.put(f"Meter reading: {result}")
                cv2.imshow('yolov8', result_img)
                cv2.waitKey(0)  # 按任意键退出显示
                cap.release()
                cv2.destroyAllWindows()
            else:
                result_queue.put("Error: Failed to get processed result.")
                cap.release()

            # 释放rknn线程池
            pool.release()

        elif command == "exit":
            print("仪表检测线程结束")
            break

# 监听命令的主线程
def command_listener(result_queue):
    while True:
        command = input("请输入命令（start_detection/exit）：")

        if command == "start_detection":
            # 发送命令给检测线程
            command_queue.put(command)

            # 等待检测结果并输出
            result = result_queue.get()
            print(f"检测结果：{result}")

        elif command == "exit":
            # 退出程序
            command_queue.put(command)
            print("程序退出")
            break
        else:
            print("未知命令，请重新输入")

if __name__ == "__main__":
    # 创建队列用于线程间的结果传递
    result_queue = queue.Queue()

    # 启动仪表检测的工作线程
    worker_thread = threading.Thread(target=meter_detection_task, args=(command_queue, result_queue))
    worker_thread.start()

    # 启动主线程监听命令
    command_listener(result_queue)

    # 等待工作线程结束
    worker_thread.join()
