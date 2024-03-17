import cv2
import mediapipe as mp
import numpy as np
from std_msgs.msg import Float64MultiArray
import rospy
import time
import math

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands

#求解关节角度
def calculate_angle(a, b, c):
    a = np.array(a)  # First
    b = np.array(b)  # Mid
    c = np.array(c)  # End

    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)

    if angle > 180.0:
        angle = 360 - angle

    return angle

#求解二维向量的角度
def vector_2d_angle(v1, v2):
    v1_x, v1_y = v1
    v2_x, v2_y = v2
    try:
        angle_ = math.degrees(math.acos((v1_x * v2_x + v1_y * v2_y) / (((v1_x**2 + v1_y**2)**0.5) * ((v2_x**2 + v2_y**2)**0.5))))
    except:
        angle_ = 65535.
    if angle_ > 180.:
        angle_ = 65535.
    return angle_

rospy.init_node('arm_synchronization_publisher')
pub = rospy.Publisher('arm_synchronization', Float64MultiArray, queue_size=10)

cap = cv2.VideoCapture(0)
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose, mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
    last_publish_time = time.time()
    while cap.isOpened():
        ret, frame = cap.read()

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False

        results_pose = pose.process(image)
        results_hands = hands.process(image)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        try:
            landmarks = results_pose.pose_landmarks.landmark

            shoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,
                        landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            elbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,
                     landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
            wrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x,
                     landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
            left_hip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x,
                        landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]

            angle1 = calculate_angle(shoulder, elbow, wrist)
            angle2 = calculate_angle(left_hip, shoulder, elbow)

            # Hand gesture recognition
            if results_hands.multi_hand_landmarks:
                for hand_landmarks in results_hands.multi_hand_landmarks:
                    hand_local = []
                    for i in range(21):
                        x = hand_landmarks.landmark[i].x * image.shape[1]
                        y = hand_landmarks.landmark[i].y * image.shape[0]
                        hand_local.append((x, y))
                    if hand_local:
                        thumb_tip = hand_local[4]
                        index_tip = hand_local[8]
                        pinky_tip = hand_local[20]
                        thumb_index_angle = vector_2d_angle(
                            (thumb_tip[0] - hand_local[0][0], thumb_tip[1] - hand_local[0][1]),
                            (index_tip[0] - hand_local[0][0], index_tip[1] - hand_local[0][1]))
                        thumb_pinky_angle = vector_2d_angle(
                            (thumb_tip[0] - hand_local[0][0], thumb_tip[1] - hand_local[0][1]),
                            (pinky_tip[0] - hand_local[0][0], pinky_tip[1] - hand_local[0][1]))   
                        cv2.putText(image, f'Thumb-Index Angle: {thumb_index_angle:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        cv2.putText(image, f'Thumb-Pinky Angle: {thumb_pinky_angle:.2f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                        current_time = time.time()
                        #每隔0.5s发布一次手臂角度
                        if current_time - last_publish_time >= 0.5:
                            angles = Float64MultiArray()
                            angles.data = [angle1, angle2, thumb_index_angle, thumb_pinky_angle]
                            pub.publish(angles)
                            last_publish_time = current_time

            cv2.putText(image, str(angle1),
                        tuple(np.multiply(elbow, [640, 480]).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                        )

        except:
            pass

        mp_drawing.draw_landmarks(image, results_pose.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                  mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                  mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                                  )

        if results_hands.multi_hand_landmarks:
            for hand_landmarks in results_hands.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        cv2.imshow('Mediapipe Feed', image)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()