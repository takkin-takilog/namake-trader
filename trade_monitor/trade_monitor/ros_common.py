from typing import TypeVar
import rclpy
from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.client import Client

SrvTypeRequest = TypeVar("SrvTypeRequest")
SrvTypeResponse = TypeVar("SrvTypeResponse")

_g_node: Node = None
_g_service_client_candle: Client = None


def set_node(node: Node) -> None:
    global _g_node
    _g_node = node


def get_node() -> Node:
    global _g_node
    return _g_node


def set_service_client_candle(client: Client) -> None:
    global _g_service_client_candle
    _g_service_client_candle = client


def get_logger():
    global _g_node
    return _g_node.get_logger()


def call_servive_sync(srv_cli: Client,
                      request: SrvTypeRequest,
                      executor: Executor=None,
                      timeout_sec: float=None
                      ) -> SrvTypeResponse:
    future = srv_cli.call_async(request)
    global _g_node
    """
    if executor is None:
        executor = SingleThreadedExecutor()
    """
    rclpy.spin_until_future_complete(_g_node, future, executor, timeout_sec)

    if future.done() is False:
        raise Exception("future.done() is False")
    if future.result() is None:
        raise Exception("future.result() is None")

    return future.result()


def call_servive_sync_candle(request: SrvTypeRequest,
                             executor: Executor=None,
                             timeout_sec: float=None
                             ) -> SrvTypeResponse:
    global _g_service_client_candle
    result = call_servive_sync(_g_service_client_candle,
                               request,
                               executor,
                               timeout_sec)
    return result
