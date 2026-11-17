"""Publish the positional error between detected_panel and the closest real panel.

Errors are detected - ground truth, expressed in the map frame, on the
x_err, y_err and z_err topics.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener

MAP_FRAME = 'map'
DETECTED_FRAME = 'detected_panel'
PANEL_FRAMES = [f'panel_{i}' for i in range(4)]


class CalcError(Node):
    """Look up detected and ground-truth panels on tf and publish their error."""

    def __init__(self):
        super().__init__('calc_error')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x_pub = self.create_publisher(Float64, 'x_err', 10)
        self.y_pub = self.create_publisher(Float64, 'y_err', 10)
        self.z_pub = self.create_publisher(Float64, 'z_err', 10)

        self.last_stamp = None
        self.create_timer(0.005, self.on_timer)

    def on_timer(self):
        # Time() asks for the latest time at which the whole map -> camera_frame
        # -> detected_panel chain is known, i.e. the newest detection. That is
        # always slightly in the past, so tf never has to extrapolate.
        try:
            detected = self.tf_buffer.lookup_transform(
                MAP_FRAME, DETECTED_FRAME, Time())
        except TransformException:
            return

        # tf keeps the last detected_panel around after detections stop, so
        # only publish when there is a detection we have not handled yet.
        stamp = Time.from_msg(detected.header.stamp)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return

        # Look up every ground-truth panel at the detection's exact timestamp.
        d = detected.transform.translation
        best = None
        for frame in PANEL_FRAMES:
            try:
                panel = self.tf_buffer.lookup_transform(MAP_FRAME, frame, stamp)
            except TransformException:
                return
            p = panel.transform.translation
            err = (d.x - p.x, d.y - p.y, d.z - p.z)
            dist = math.sqrt(err[0] ** 2 + err[1] ** 2 + err[2] ** 2)
            if best is None or dist < best[0]:
                best = (dist, err)

        self.last_stamp = stamp
        _, (x_err, y_err, z_err) = best
        self.x_pub.publish(Float64(data=x_err))
        self.y_pub.publish(Float64(data=y_err))
        self.z_pub.publish(Float64(data=z_err))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CalcError())


if __name__ == '__main__':
    main()
