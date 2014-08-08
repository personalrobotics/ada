class AdaPyException(Exception):
    """
    Generic AdaPy exception.
    """

class TrajectoryAborted(AdaPyException):
    """
    Trajectory was aborted.
    """

class TrajectoryStalled(AdaPyException):
    """
    Trajectory stalled.
    """

class SynchronizationException(AdaPyException):
    """
    Controller synchronization failed.
    """
