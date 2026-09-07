
#!/usr/bin/env python3

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

import message_filters

from inference import get_model

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class AssistanceRobotVision(Node):

    def __init__(self):

        super().__init__("yolo_picker")

        # =========================================================
        # CAMERA INTRINSICS
        # =========================================================

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # =========================================================
        # ROBOFLOW API KEY
        # =========================================================
        #
        # Put your REAL Roboflow API key here.
        #
        # Do NOT send the key to me or post it publicly.
        #
        # =========================================================

        api_key = "eXOnVDfSMNPGyakDKRE7"

        # =========================================================
        # LOAD YOLO MODEL
        # =========================================================

        self.get_logger().info(
            "Loading Roboflow model..."
        )

        self.model = get_model(
            model_id="assistance-robot/1",
            api_key=api_key
        )

        self.get_logger().info(
            "Roboflow model loaded successfully."
        )

        # =========================================================
        # TF2
        # =========================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =========================================================
        # PUBLISHER
        # =========================================================

        self.position_pub = self.create_publisher(
            PointStamped,
            "/detected_object/position",
            10
        )

        # =========================================================
        # CAMERA INFO
        # =========================================================

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.camera_info_callback,
            10
        )

        # =========================================================
        # RGB + DEPTH
        # =========================================================

        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            "/camera/color/image_raw"
        )

        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            "/camera/depth/image_rect_raw"
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [
                self.rgb_sub,
                self.depth_sub
            ],
            queue_size=10,
            slop=0.1
        )

        self.sync.registerCallback(
            self.rgb_depth_callback
        )

        self.get_logger().info(
            "Waiting for RGB + Depth + CameraInfo..."
        )

    # =============================================================
    # CAMERA INFO
    # =============================================================

    def camera_info_callback(self, msg):

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])

        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

        self.get_logger().info(
            "Camera intrinsics received: "
            f"fx={self.fx:.3f}, "
            f"fy={self.fy:.3f}, "
            f"cx={self.cx:.3f}, "
            f"cy={self.cy:.3f}"
        )

        self.destroy_subscription(
            self.camera_info_sub
        )

    # =============================================================
    # RGB IMAGE CONVERSION
    # =============================================================

    def convert_rgb_to_cv(self, rgb_msg):

        if rgb_msg.encoding == "rgb8":

            image = np.frombuffer(
                rgb_msg.data,
                dtype=np.uint8
            ).reshape(
                rgb_msg.height,
                rgb_msg.width,
                3
            )

            image = cv2.cvtColor(
                image,
                cv2.COLOR_RGB2BGR
            )

            return image

        elif rgb_msg.encoding == "bgr8":

            image = np.frombuffer(
                rgb_msg.data,
                dtype=np.uint8
            ).reshape(
                rgb_msg.height,
                rgb_msg.width,
                3
            )

            return image

        elif rgb_msg.encoding == "rgba8":

            image = np.frombuffer(
                rgb_msg.data,
                dtype=np.uint8
            ).reshape(
                rgb_msg.height,
                rgb_msg.width,
                4
            )

            image = cv2.cvtColor(
                image,
                cv2.COLOR_RGBA2BGR
            )

            return image

        elif rgb_msg.encoding == "bgra8":

            image = np.frombuffer(
                rgb_msg.data,
                dtype=np.uint8
            ).reshape(
                rgb_msg.height,
                rgb_msg.width,
                4
            )

            image = cv2.cvtColor(
                image,
                cv2.COLOR_BGRA2BGR
            )

            return image

        else:

            raise ValueError(
                "Unsupported RGB encoding: "
                f"{rgb_msg.encoding}"
            )

    # =============================================================
    # DEPTH IMAGE CONVERSION
    # =============================================================

    def convert_depth_to_numpy(self, depth_msg):

        # Gazebo is currently publishing 32FC1
        # and the values are already in meters.

        if depth_msg.encoding == "32FC1":

            depth_image = np.frombuffer(
                depth_msg.data,
                dtype=np.float32
            ).reshape(
                depth_msg.height,
                depth_msg.width
            )

            depth_scale = 1.0

            return depth_image, depth_scale

        # Some cameras publish depth as 16UC1.
        # Usually this is millimeters.

        elif depth_msg.encoding == "16UC1":

            depth_image = np.frombuffer(
                depth_msg.data,
                dtype=np.uint16
            ).reshape(
                depth_msg.height,
                depth_msg.width
            )

            depth_scale = 0.001

            return depth_image, depth_scale

        else:

            raise ValueError(
                "Unsupported depth encoding: "
                f"{depth_msg.encoding}"
            )

    # =============================================================
    # GET OBJECT CENTER
    # =============================================================

    def get_object_center(self, obj):

        # ---------------------------------------------------------
        # First choice:
        # Roboflow prediction normally contains x and y.
        # ---------------------------------------------------------

        x = getattr(
            obj,
            "x",
            None
        )

        y = getattr(
            obj,
            "y",
            None
        )

        if x is not None and y is not None:

            try:

                u = int(
                    round(
                        float(x)
                    )
                )

                v = int(
                    round(
                        float(y)
                    )
                )

                return u, v

            except (
                TypeError,
                ValueError
            ):

                pass

        # ---------------------------------------------------------
        # Fallback:
        # use segmentation polygon
        # ---------------------------------------------------------

        mask = getattr(
            obj,
            "mask",
            None
        )

        if mask is None:

            return None

        points = mask

        if hasattr(mask, "points"):

            points = mask.points

        coordinates = []

        try:

            for point in points:

                px = getattr(
                    point,
                    "x",
                    None
                )

                py = getattr(
                    point,
                    "y",
                    None
                )

                if (
                    px is not None
                    and py is not None
                ):

                    coordinates.append(
                        (
                            float(px),
                            float(py)
                        )
                    )

        except TypeError:

            return None

        if len(coordinates) == 0:

            return None

        xs = np.array(
            [
                point[0]
                for point in coordinates
            ],
            dtype=np.float32
        )

        ys = np.array(
            [
                point[1]
                for point in coordinates
            ],
            dtype=np.float32
        )

        u = int(
            round(
                float(
                    np.mean(xs)
                )
            )
        )

        v = int(
            round(
                float(
                    np.mean(ys)
                )
            )
        )

        return u, v

    # =============================================================
    # GET DEPTH
    # =============================================================

    def get_depth(
        self,
        depth_image,
        u,
        v,
        depth_scale
    ):

        height, width = depth_image.shape

        # Keep pixel inside image

        u = max(
            0,
            min(
                width - 1,
                u
            )
        )

        v = max(
            0,
            min(
                height - 1,
                v
            )
        )

        # ---------------------------------------------------------
        # Use a small 5x5 window instead of a single pixel.
        # Median makes the measurement more stable.
        # ---------------------------------------------------------

        radius = 2

        u_min = max(
            0,
            u - radius
        )

        u_max = min(
            width,
            u + radius + 1
        )

        v_min = max(
            0,
            v - radius
        )

        v_max = min(
            height,
            v + radius + 1
        )

        roi = depth_image[
            v_min:v_max,
            u_min:u_max
        ]

        valid_depth = roi[
            np.isfinite(roi)
            &
            (roi > 0)
        ]

        if len(valid_depth) == 0:

            return None

        depth_value = float(
            np.median(valid_depth)
        )

        depth_meters = (
            depth_value
            * depth_scale
        )

        if (
            not np.isfinite(depth_meters)
            or depth_meters <= 0
        ):

            return None

        return depth_meters

    # =============================================================
    # RGB + DEPTH CALLBACK
    # =============================================================

    def rgb_depth_callback(
        self,
        rgb_msg,
        depth_msg
    ):

        try:

            # =====================================================
            # CHECK CAMERA INTRINSICS
            # =====================================================

            if (
                self.fx is None
                or self.fy is None
                or self.cx is None
                or self.cy is None
            ):

                self.get_logger().warn(
                    "Waiting for camera intrinsics..."
                )

                return

            # =====================================================
            # CONVERT RGB
            # =====================================================

            cv_image = self.convert_rgb_to_cv(
                rgb_msg
            )

            # =====================================================
            # CONVERT DEPTH
            # =====================================================

            depth_image, depth_scale = (
                self.convert_depth_to_numpy(
                    depth_msg
                )
            )

            # =====================================================
            # YOLO INFERENCE
            # =====================================================

            results = self.model.infer(
                cv_image
            )

            if not results:

                return

            # =====================================================
            # EXTRACT PREDICTIONS
            # =====================================================

            all_predictions = []

            for response in results:

                predictions = getattr(
                    response,
                    "predictions",
                    []
                )

                if predictions:

                    all_predictions.extend(
                        predictions
                    )

            if len(all_predictions) == 0:

                return

            # =====================================================
            # SELECT HIGHEST CONFIDENCE OBJECT
            # =====================================================

            best_object = None
            best_confidence = -1.0

            for obj in all_predictions:

                confidence = getattr(
                    obj,
                    "confidence",
                    0.0
                )

                try:

                    confidence = float(
                        confidence
                    )

                except (
                    TypeError,
                    ValueError
                ):

                    confidence = 0.0

                if confidence > best_confidence:

                    best_confidence = confidence
                    best_object = obj

            if best_object is None:

                return

            # =====================================================
            # OBJECT CLASS
            # =====================================================

            object_class = getattr(
                best_object,
                "class_name",
                None
            )

            if object_class is None:

                object_class = getattr(
                    best_object,
                    "class",
                    "unknown"
                )

            # =====================================================
            # OBJECT CENTER
            # =====================================================

            center = self.get_object_center(
                best_object
            )

            if center is None:

                self.get_logger().warn(
                    "Could not determine object center."
                )

                return

            u, v = center

            # =====================================================
            # KEEP PIXEL INSIDE DEPTH IMAGE
            # =====================================================

            depth_height, depth_width = (
                depth_image.shape
            )

            u = max(
                0,
                min(
                    depth_width - 1,
                    u
                )
            )

            v = max(
                0,
                min(
                    depth_height - 1,
                    v
                )
            )

            # =====================================================
            # GET DEPTH
            # =====================================================

            Z = self.get_depth(
                depth_image,
                u,
                v,
                depth_scale
            )

            if Z is None:

                self.get_logger().warn(
                    f"No valid depth at "
                    f"pixel ({u}, {v})"
                )

                return

            # =====================================================
            # PIXEL -> CAMERA 3D
            # =====================================================
            #
            # X = (u - cx) * Z / fx
            #
            # Y = (v - cy) * Z / fy
            #
            # Z = depth
            #
            # =====================================================

            X = (
                (u - self.cx)
                * Z
                / self.fx
            )

            Y = (
                (v - self.cy)
                * Z
                / self.fy
            )

            # =====================================================
            # CREATE POINT IN CAMERA FRAME
            # =====================================================

            point_camera = PointStamped()

            point_camera.header.stamp = (
                depth_msg.header.stamp
            )

            point_camera.header.frame_id = (
                depth_msg.header.frame_id
            )

            point_camera.point.x = float(X)
            point_camera.point.y = float(Y)
            point_camera.point.z = float(Z)

            camera_frame = (
                depth_msg.header.frame_id
            )

            # =====================================================
            # CAMERA FRAME -> BASE FRAME
            # =====================================================

            try:

                transform = (
                    self.tf_buffer.lookup_transform(
                        "base_link",
                        camera_frame,
                        point_camera.header.stamp,
                        timeout=rclpy.duration.Duration(
                            seconds=0.2
                        )
                    )
                )

            except Exception:

                # -------------------------------------------------
                # If exact timestamp is unavailable,
                # use latest available TF.
                # -------------------------------------------------

                try:

                    transform = (
                        self.tf_buffer.lookup_transform(
                            "base_link",
                            camera_frame,
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(
                                seconds=0.2
                            )
                        )
                    )

                except Exception as e:

                    self.get_logger().warn(
                        "TF unavailable: "
                        f"{e}"
                    )

                    return

            # =====================================================
            # TRANSFORM POINT
            # =====================================================

            point_base = (
                do_transform_point(
                    point_camera,
                    transform
                )
            )

            # =====================================================
            # PUBLISH BASE FRAME POSITION
            # =====================================================

            self.position_pub.publish(
                point_base
            )

            # =====================================================
            # PRINT RESULT
            # =====================================================

            self.get_logger().info(
                f"Detected: {object_class} "
                f"| confidence="
                f"{best_confidence:.2f} "
                f"| pixel=({u}, {v}) "
                f"| depth={Z:.3f} m "
                f"| camera XYZ=("
                f"{X:.3f}, "
                f"{Y:.3f}, "
                f"{Z:.3f}) m "
                f"| base XYZ=("
                f"{point_base.point.x:.3f}, "
                f"{point_base.point.y:.3f}, "
                f"{point_base.point.z:.3f}) m"
            )

        except Exception as e:

            self.get_logger().error(
                f"Vision error: {e}"
            )

    # =============================================================
    # MAIN
    # =============================================================


def main(args=None):

    rclpy.init(
        args=args
    )

    node = AssistanceRobotVision()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()

