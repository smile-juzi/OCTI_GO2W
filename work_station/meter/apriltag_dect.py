import cv2
import apriltag

options = apriltag.DetectorOptions(families="tag36h11")
# 初始化摄像头和 AprilTag 检测器
cap = cv2.VideoCapture(0)



detector = apriltag.Detector(options)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 将图像转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)

    # 检测 AprilTag
    tags = detector.detect(gray)

    # 遍历所有检测到的标签，并在图像中标注
    for tag in tags:
        # 获取标签的角点和中心点
        corners = tag.corners
        center = (int(tag.center[0]), int(tag.center[1]))

        # 绘制标签的外部矩形框
        for i in range(4):
            pt1 = (int(corners[i][0]), int(corners[i][1]))
            pt2 = (int(corners[(i+1) % 4][0]), int(corners[(i+1) % 4][1]))
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        # 在标签中心绘制一个小圆点
        cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # 标注标签的 ID
        tag_id = str(tag.tag_id)
        cv2.putText(frame, f"ID: {tag_id}", (center[0] - 10, center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # 显示结果
    cv2.imshow("Frame", frame)

    if len(tags) >= 2:  # 至少需要两个 AprilTag 才能确定仪表区域
        # 假设只处理两个标签，分别是对角的两个标签
        tag1, tag2 = tags[0], tags[1]

        # 取出两个标签的中心位置
        center1 = (int(tag1.center[0]), int(tag1.center[1]))
        center2 = (int(tag2.center[0]), int(tag2.center[1]))

        # 计算对角的两个点
        top_left = (min(center1[0], center2[0]), min(center1[1], center2[1]))
        bottom_right = (max(center1[0], center2[0]), max(center1[1], center2[1]))

        # 在图像上绘制矩形，表示仪表的区域
        cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

        # 提取仪表的区域
        instrument_roi = frame[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]

        # 显示裁剪出的仪表区域
        if instrument_roi.size > 0:
            cv2.imshow("Instrument", instrument_roi)

    # 显示结果
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
