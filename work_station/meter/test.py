import cv2
from rknnpool import rknnPoolExecutor
from func import myFunc
from read_meter import MeterReader
from dect_meter import capture_meter_frame
import numpy as np

# 读取图片
cap = cv2.VideoCapture(0)
image, category = capture_meter_frame(cap)

# img_path = '1008.jpg'
# img_path = '1008.jpg'
# image = cv2.imread(img_path)

if image is None:
    print("Error: Could not read the image.")
    exit(-1)

# 模型路径
modelPath = "./rknnModel/yolov8_seg_newer.rknn"
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
    # 显示结果
    print(image.shape)  # 确认 image 的形状是否正确

    result_img, pointer_mask, scale_mask = result
    pointer_mask_uint8 = pointer_mask.astype(np.uint8)
    scale_mask_uint8 = scale_mask.astype(np.uint8)
    pointer_mask_combined = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
    scale_mask_combined = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)

    # Iterate through each scale mask
    for i in range(scale_mask.shape[0]):
        scale_mask_current = scale_mask[i]  # Extract current scale mask
        if scale_mask_current.dtype == np.bool_:
            scale_mask_current = scale_mask_current.astype(np.uint8) * 255  # Convert boolean to uint8

        # Resize scale mask to match image size
        scale_mask_resized = cv2.resize(scale_mask_current, (image.shape[1], image.shape[0]),
                                        interpolation=cv2.INTER_NEAREST)
        # Combine masks using bitwise OR
        scale_mask_combined = cv2.bitwise_or(scale_mask_combined, scale_mask_resized)
    for i in range(pointer_mask.shape[0]):
        pointer_mask_current = pointer_mask[i]  # Extract current scale mask
        if pointer_mask_current.dtype == np.bool_:
            pointer_mask_current = pointer_mask_current.astype(np.uint8) * 255  # Convert boolean to uint8

        # Resize scale mask to match image size
        pointer_mask_resized = cv2.resize(pointer_mask_current, (image.shape[1], image.shape[0]),
                                        interpolation=cv2.INTER_NEAREST)
        # Combine masks using bitwise OR
        pointer_mask_combined = cv2.bitwise_or(pointer_mask_combined, pointer_mask_resized)

    print("Scale mask shape:", scale_mask_combined.shape)
    print("Target size:", (image.shape[1], image.shape[0]))

    point_mask_resized = cv2.resize(pointer_mask_combined, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
    dail_mask_resized = cv2.resize(scale_mask_combined, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
    # 使用 MeterReader 进行读数计算
    meter_reader = MeterReader(image)
    result = meter_reader(point_mask_resized, dail_mask_resized)
    print(f"Meter reading: {result}")
    cv2.imshow('yolov8', result_img)
    cv2.waitKey(0)  # 按任意键退出显示
    cap.release()
    cv2.destroyAllWindows()
else:
    print("Error: Failed to get processed result.")
    cap.release()