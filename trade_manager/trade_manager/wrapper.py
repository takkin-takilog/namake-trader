from typing import TypeVar
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from rclpy.qos import QoSProfile
from rclpy.callback_groups import CallbackGroup
from .exception import RosServiceErrorException

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")


class Future:
    """
    Future class.
    """

    def __init__(self, future, end_time: float | None = None) -> None:
        self._future = future
        self._end_time = end_time

    def has_timed_out(self) -> bool:
        if (self._end_time is not None) and (self._end_time < time.monotonic()):
            return True
        return False

    def done(self) -> bool:
        return self._future.done()

    def result(self):
        return self._future.result()


class RosServiceClient:
    """
    Ros service client class.
    """

    def __init__(
        self,
        node: Node,
        srv_type,
        srv_name: str,
        *,
        qos_profile: QoSProfile = qos_profile_services_default,
        callback_group: CallbackGroup = None,
        use_wait_for_service: bool = True
    ) -> None:

        self._node = node
        self.logger = self._node.get_logger()
        cli = self._node.create_client(
            srv_type, srv_name, qos_profile=qos_profile, callback_group=callback_group
        )
        if use_wait_for_service:
            while not cli.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    raise RosServiceErrorException(
                        "Interrupted while waiting for service."
                    )
                self.logger.info("Waiting for [{}] service...".format(srv_name))
        self._cli = cli
        self._srv_name = srv_name

    def call(self, request: SrvTypeRequest, timeout_sec: float | None = None):

        if not self._cli.service_is_ready():
            msg = "Server [{}] Not Ready.".format(self._srv_name)
            raise RosServiceErrorException(msg)

        try:
            future = self._cli.call_async(request)
        except BaseException as err:
            self.logger.error("{}".format(err))
            msg = "Call ROS Service [{}] Failed.".format(self._srv_name)
            raise RosServiceErrorException(msg) from err

        if (timeout_sec is not None) and (0 < timeout_sec):
            enable_timeout = True
            end_time = time.monotonic() + timeout_sec
        else:
            enable_timeout = False

        while not future.done():
            if enable_timeout and end_time < time.monotonic():
                msg = "ROS Service [{}] Timeout.".format(self._srv_name)
                raise RosServiceErrorException(msg)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.0)
            self.logger.info("Waiting service [{}] has done...".format(self._srv_name))

        result = future.result()
        if result is None:
            msg = "ROS Service [{}] Future's Result is None.".format(self._srv_name)
            raise RosServiceErrorException(msg)

        return result

    def call_async(
        self, request: SrvTypeRequest, timeout_sec: float | None = None
    ) -> Future:

        if not self._cli.service_is_ready():
            msg = "Server [{}] Not Ready.".format(self._srv_name)
            raise RosServiceErrorException(msg)

        try:
            future = self._cli.call_async(request)
        except BaseException as err:
            self.logger.error("{}".format(err))
            msg = "Call ROS Service [{}] Failed.".format(self._srv_name)
            raise RosServiceErrorException(msg) from err

        if (timeout_sec is not None) and (0 < timeout_sec):
            end_time = time.monotonic() + timeout_sec
        else:
            end_time = None

        return Future(future, end_time)
