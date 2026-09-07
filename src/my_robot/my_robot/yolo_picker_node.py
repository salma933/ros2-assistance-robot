#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

import message_filters

import numpy as np
import cv2

from inference import get_model

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class AssistanceRobotVision(Node):

    def __init__(self):
        super().__init__('assistance_robot_vision')

        self.get_logger().info(
            "Starting Assistance Robot Vision..."
        )

        # =========================================================
        # ROBoflow MODEL
        # =========================================================

        self.model = get_model(
            model_id="assistance-robot/1",
            api_key="eXOnVDfSMNPGyakDKRE7"
        )

        self.get_logger().info(
            "Roboflow model loaded successfully."
        )

        # =========================================================
        # CAMERA INTRINSICS
        # =========================================================

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.camera_info_callback,
            10
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
        # PUBLISH DETECTED OBJECT POSITION
        # =========================================================

        self.position_pub = self.create_publisher(
            PointStamped,
            "/detected_object/position",
            10
        )

        # =========================================================
        # RGB SUBSCRIBER
        # =========================================================

        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            "/camera/color/image_raw"
        )

        # =========================================================
        # DEPTH SUBSCRIBER
        # =========================================================

        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            "/camera/depth/image_rect_raw"
        )

        # =========================================================
        # SYNCHRONIZE RGB + DEPTH
        # =========================================================

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

        self.fx = msg.k[0]
        self.fy = msg.k[4]

        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.get_logger().info(
            f"Camera intrinsics received: "
            f"fx={self.fx:.3f}, "
            f"fy={self.fy:.3f}, "
            f"cx={self.cx:.3f}, "
            f"cy={self.cy:.3f}"
        )

        # We only need CameraInfo once
        self.destroy_subscription(
            self.camera_info_sub
        )

    # =============================================================
    # RGB + DEPTH CALLBACK
    # =============================================================

    def rgb_depth_callback(
        self,
        rgb_msg,
        depth_msg
    ):

        try:

            # -----------------------------------------------------
            # Make sure camera info exists
            # -----------------------------------------------------

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

            # -----------------------------------------------------
            # RGB -> OpenCV
            # -----------------------------------------------------

            if rgb_msg.encoding == "rgb8":

                cv_image = np.frombuffer(
                    rgb_msg.data,
                    dtype=np.uint8
                ).reshape(
                    rgb_msg.height,
                    rgb_msg.width,
                    3
                )

                cv_image = cv2.cvtColor(
                    cv_image,
                    cv2.COLOR_RGB2BGR
                )

            elif rgb_msg.encoding == "bgr8":

                cv_image = np.frombuffer(
                    rgb_msg.data,
                    dtype=np.uint8
                ).reshape(
                    rgb_msg.height,
                    rgb_msg.width,
                    3
                )

            elif rgb_msg.encoding == "rgba8":

                cv_image = np.frombuffer(
                    rgb_msg.data,
                    dtype=np.uint8
                ).reshape(
                    rgb_msg.height,
                    rgb_msg.width,
                    4
                )

                cv_image = cv2.cvtColor(
                    cv_image,
                    cv2.COLOR_RGBA2BGR
                )

            elif rgb_msg.encoding == "bgra8":

                cv_image = np.frombuffer(
                    rgb_msg.data,
                    dtype=np.uint8
                ).reshape(
                    rgb_msg.height,
                    rgb_msg.width,
                    4
                )

                cv_image = cv2.cvtColor(
                    cv_image,
                    cv2.COLOR_BGRA2BGR
                )

            else:

                self.get_logger().error(
                    f"Unsupported RGB encoding: "
                    f"{rgb_msg.encoding}"
                )

                return

            # -----------------------------------------------------
            # DEPTH -> NumPy
            # -----------------------------------------------------

            if depth_msg.encoding == "32FC1":

                depth_image = np.frombuffer(
                    depth_msg.data,
                    dtype=np.float32
                ).reshape(
                    depth_msg.height,
                    depth_msg.width
                )

                depth_scale = 1.0

            elif depth_msg.encoding == "16UC1":

                depth_image = np.frombuffer(
                    depth_msg.data,
                    dtype=np.uint16
                ).reshape(
                    depth_msg.height,
                    depth_msg.width
                )

                depth_scale = 0.001

            else:

                self.get_logger().error(
                    f"Unsupported depth encoding: "
                    f"{depth_msg.encoding}"
                )

                return

            # -----------------------------------------------------
            # RUN YOLO
            # -----------------------------------------------------

            results = self.model.infer(
                cv_image
            )

            # =====================================================
            # DIAGNOSTIC
            # =====================================================

            print("\n===============================")
            print("RESULT TYPE:")
            print(type(results))

            print("\nRESULT ATTRS:")
            print(dir(results))

            print("\nRESULT LENGTH:")
            print(len(results))

            # -----------------------------------------------------
            # If nothing returned
            # -----------------------------------------------------

            if len(results) == 0:

                print(
                    "No inference response returned."
                )

                return

            # -----------------------------------------------------
            # First response
            # -----------------------------------------------------

            response = results[0]

            print("\n===============================")
            print("FIRST RESPONSE TYPE:")
            print(type(response))

            print("\nFIRST RESPONSE ATTRS:")
            print(dir(response))

            # -----------------------------------------------------
            # Predictions
            # -----------------------------------------------------

            if hasattr(response, "predictions"):

                predictions = response.predictions

                print("\n===============================")
                print("PREDICTIONS:")
                print(predictions)

                print("\nPREDICTIONS TYPE:")
                print(type(predictions))

                print("\nPREDICTIONS LENGTH:")
                print(len(predictions))

                # -------------------------------------------------
                # No objects
                # -------------------------------------------------

                if len(predictions) == 0:

                    print(
                        "No objects detected."
                    )

                    return

                # -------------------------------------------------
                # First detected object
                # -------------------------------------------------

                obj = predictions[0]

                print("\n===============================")
                print("FIRST OBJECT TYPE:")
                print(type(obj))

                print("\nFIRST OBJECT ATTRS:")
                print(dir(obj))

                print("\nFIRST OBJECT:")
                print(obj)

                print("===============================\n")

                # -------------------------------------------------
                # STOP HERE FOR DIAGNOSTIC
                # -------------------------------------------------

                return

            else:

                print(
                    "Response has no 'predictions' attribute."
                )

                return

        except Exception as e:

            self.get_logger().error(
                f"Vision error: {e}"
            )


# =================================================================
# MAIN
# =================================================================

def main(args=None):

    rclpy.init(args=args)

    node = AssistanceRobotVision()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()