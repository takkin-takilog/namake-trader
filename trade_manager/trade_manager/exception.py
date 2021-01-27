

class InitializerErrorException(Exception):
    """Raised when an exception occurs during class initialization."""

    def __init__(self, *args):
        Exception.__init__(self, "Error occurs during class initialization", *args)
