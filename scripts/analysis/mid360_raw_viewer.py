#!/usr/bin/env python3
"""Display-only CustomMsg -> PointCloud2 adapter; no geometry processing."""
import struct

POINT = struct.Struct('<ffffBBxxI')

def pack_points(points):
    data = bytearray(len(points) * POINT.size)
    for i, point in enumerate(points):
        POINT.pack_into(data, i * POINT.size, point.x, point.y, point.z,
                        float(point.reflectivity), point.tag, point.line,
                        point.offset_time)
    return bytes(data)

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import PointCloud2, PointField

    class Viewer(Node):
        def __init__(self):
            super().__init__('mid360_raw_display_adapter')
            self.pub = self.create_publisher(PointCloud2, '/mid360/raw_points', qos_profile_sensor_data)
            self.sub = self.create_subscription(CustomMsg, '/livox/lidar', self.receive, qos_profile_sensor_data)
            self.received = 0
            self.last_received = 0
            self.timer = self.create_timer(5., self.status)
            self.get_logger().info('Waiting for /livox/lidar. Display output: /mid360/raw_points. No nodes or parameters are changed.')

        def status(self):
            if self.received == self.last_received:
                self.get_logger().warning('No new raw cloud in the last 5 seconds; check the existing LiDAR process. This adapter does not start the driver.')
            self.last_received = self.received

        def receive(self, msg):
            cloud = PointCloud2()
            cloud.header = msg.header
            cloud.height = 1
            cloud.width = len(msg.points)
            cloud.fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                            for name, offset, kind in (
                                ('x',0,PointField.FLOAT32), ('y',4,PointField.FLOAT32),
                                ('z',8,PointField.FLOAT32), ('intensity',12,PointField.FLOAT32),
                                ('tag',16,PointField.UINT8), ('line',17,PointField.UINT8),
                                ('offset_time',20,PointField.UINT32))]
            cloud.is_bigendian = False
            cloud.point_step = POINT.size
            cloud.row_step = cloud.point_step * cloud.width
            cloud.is_dense = False
            cloud.data = pack_points(msg.points)
            self.pub.publish(cloud)
            self.received += 1
            if self.received == 1:
                self.get_logger().info(f'First raw cloud: frame={msg.header.frame_id!r}, points={cloud.width}. Use this frame as RViz Fixed Frame.')

    rclpy.init()
    node = Viewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
