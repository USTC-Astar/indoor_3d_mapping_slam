#!/usr/bin/env python3
import os
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YoloCatDetector:
    def __init__(self):
        if YOLO is None:
            raise rospy.ROSException(
                "ultralytics is not installed. Install it with: "
                "python3 -m pip install --user ultralytics"
            )

        self.image_topic = rospy.get_param("~image_topic", "/robot_view/image_raw")
        self.annotated_topic = rospy.get_param("~annotated_topic", "/robot_view/yolo/image_annotated")
        self.status_topic = rospy.get_param("~status_topic", "/robot_view/yolo/status")
        self.model_path = os.path.expanduser(rospy.get_param("~model_path", "yolov8n.pt"))
        self.conf_threshold = float(rospy.get_param("~confidence_threshold", 0.25))
        self.inference_rate = max(0.1, float(rospy.get_param("~inference_rate", 4.0)))
        self.hold_seconds = max(0.0, float(rospy.get_param("~hold_seconds", 0.6)))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "auto")
        self.cat_class_id = int(rospy.get_param("~cat_class_id", 15))
        self.show_no_detection = bool(rospy.get_param("~show_no_detection", True))

        rospy.loginfo("Loading YOLO cat detector model: %s", self.model_path)
        self.model = YOLO(self.model_path)
        self.latest_detections = []
        self.latest_inference_wall_time = 0.0
        self.min_inference_period = 1.0 / self.inference_rate

        self.image_pub = rospy.Publisher(self.annotated_topic, Image, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo(
            "YOLO cat detector subscribed to %s and publishing annotated images on %s",
            self.image_topic,
            self.annotated_topic,
        )

    def image_callback(self, msg):
        try:
            frame = self.image_msg_to_bgr(msg)
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, "Could not convert camera image: %s", exc)
            return

        now = time.monotonic()
        if now - self.latest_inference_wall_time >= self.min_inference_period:
            self.latest_detections = self.detect_cats(frame)
            self.latest_inference_wall_time = now
            self.publish_status(self.latest_detections)

        detections_are_fresh = now - self.latest_inference_wall_time <= self.hold_seconds
        if detections_are_fresh:
            annotated = self.draw_detections(frame.copy(), self.latest_detections)
        else:
            annotated = frame.copy()

        if self.show_no_detection and not self.latest_detections:
            self.draw_status_banner(annotated, "YOLO cat: no detection", (70, 70, 70))

        self.image_pub.publish(self.bgr_to_image_msg(annotated, msg.header))

    def image_msg_to_bgr(self, msg):
        encoding = msg.encoding.lower()
        if encoding in ("bgr8", "rgb8"):
            channels = 3
        elif encoding in ("bgra8", "rgba8"):
            channels = 4
        elif encoding in ("mono8", "8uc1"):
            channels = 1
        else:
            raise ValueError("unsupported encoding '%s'" % msg.encoding)

        row_bytes = msg.width * channels
        if msg.step < row_bytes:
            raise ValueError("invalid image step %d for %s image width %d" % (msg.step, msg.encoding, msg.width))

        data = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.step
        if data.size < expected:
            raise ValueError("image data is shorter than expected")

        rows = data[:expected].reshape((msg.height, msg.step))
        packed = rows[:, :row_bytes]
        if channels == 1:
            image = packed.reshape((msg.height, msg.width))
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        image = packed.reshape((msg.height, msg.width, channels)).copy()
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image

    def bgr_to_image_msg(self, frame, header):
        frame = np.ascontiguousarray(frame)
        height, width = frame.shape[:2]
        out_msg = Image()
        out_msg.header = header
        out_msg.height = height
        out_msg.width = width
        out_msg.encoding = "bgr8"
        out_msg.is_bigendian = 0
        out_msg.step = width * 3
        out_msg.data = frame.tobytes()
        return out_msg

    def detect_cats(self, frame):
        kwargs = {
            "classes": [self.cat_class_id],
            "conf": self.conf_threshold,
            "imgsz": self.imgsz,
            "verbose": False,
        }
        if self.device and self.device != "auto":
            kwargs["device"] = self.device

        try:
            result = self.model.predict(frame, **kwargs)[0]
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "YOLO inference failed: %s", exc)
            return []

        detections = []
        if result.boxes is None:
            return detections

        for box in result.boxes:
            class_id = int(box.cls[0].item())
            if class_id != self.cat_class_id:
                continue
            confidence = float(box.conf[0].item())
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
            detections.append((x1, y1, x2, y2, confidence))

        detections.sort(key=lambda item: item[4], reverse=True)
        return detections

    def draw_detections(self, frame, detections):
        if not detections:
            return frame

        best = detections[0]
        self.draw_status_banner(frame, "YOLO cat: %.2f" % best[4], (24, 132, 66))

        for x1, y1, x2, y2, confidence in detections:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (40, 220, 70), 2)
            label = "cat %.2f" % confidence
            (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.62, 2)
            label_y = max(y1, text_h + 8)
            cv2.rectangle(
                frame,
                (x1, label_y - text_h - baseline - 8),
                (x1 + text_w + 10, label_y + baseline - 2),
                (40, 220, 70),
                -1,
            )
            cv2.putText(
                frame,
                label,
                (x1 + 5, label_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 0, 0),
                2,
                cv2.LINE_AA,
            )
        return frame

    def draw_status_banner(self, frame, text, color):
        height, width = frame.shape[:2]
        font_scale = 0.68
        thickness = 2
        (text_w, text_h), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        x = 12
        y = height - 14
        cv2.rectangle(frame, (x - 7, y - text_h - baseline - 8), (min(width - 6, x + text_w + 8), y + baseline), color, -1)
        cv2.putText(frame, text, (x, y - 4), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

    def publish_status(self, detections):
        if detections:
            best = detections[0]
            self.status_pub.publish("cat %.3f bbox=%d,%d,%d,%d" % (best[4], best[0], best[1], best[2], best[3]))
        else:
            self.status_pub.publish("no cat")


def main():
    rospy.init_node("yolo_cat_detector")
    YoloCatDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
