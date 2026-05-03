import os
import threading
import http.server
import functools
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix

PACKAGE_NAME = "ros2_lingua_mock"
DEFAULT_PORT = 8080


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == "/" or self.path == "":
            self.path = "/index.html"
        super().do_GET()


class DashboardServerNode(Node):
    def __init__(self):
        super().__init__("dashboard_server_node")
        self.declare_parameter("port", DEFAULT_PORT)
        port = self.get_parameter("port").value

        # Find dashboard dir from installed package prefix
        prefix = get_package_prefix(PACKAGE_NAME)
        dashboard_dir = os.path.join(prefix, "lib", PACKAGE_NAME, "dashboard")

        if not os.path.isdir(dashboard_dir):
            self.get_logger().error(f"Dashboard dir not found: {dashboard_dir}")
            return

        handler = functools.partial(DashboardHandler, directory=dashboard_dir)
        self._server = http.server.HTTPServer(("", port), handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"\n"
            f"  ╔══════════════════════════════════════╗\n"
            f"  ║   ros2_lingua dashboard running       ║\n"
            f"  ║   http://localhost:{port}               ║\n"
            f"  ╚══════════════════════════════════════╝\n"
        )

    def destroy_node(self):
        if hasattr(self, '_server'):
            self._server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
