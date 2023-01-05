class InitializerErrorException(Exception):
    """Raised when an exception occurs during class initialization."""

    def __init__(self, *args: object) -> None:
        Exception.__init__(self, "Error occurs during class initialization", *args)


class RosServiceErrorException(Exception):
    """Raised when an exception occurs during ROS service."""

    def __init__(self, *args: object) -> None:
        Exception.__init__(self, "Error occurs during ROS service", *args)
