import logging


class ControlException(RuntimeError):
    pass


class InternalError(ControlException):
    pass


class TrajectoryExecutionFailed(ControlException):
    pass

class TimeoutError(Exception):
    pass

class CancelledError(Exception):
    pass

class Future(object):
    logger = logging.getLogger('future')

    def __init__(self):
        from Queue import Queue
        from threading import Condition, Lock

        self._is_done = False
        self._is_error = False
        self._is_cancelled = False

        self._handle = None
        self._result = None

        self._lock = Lock()
        self._condition = Condition(self._lock)
        self._callbacks = []

    def done(self):
        """ Return True if the call was cancelled or finished running. """
        with self._lock:
            return self._is_done

    def cancel(self):
        """ Attempt to cancel the call. """
        raise NotImplementedError('Cancelling is not supported.')

    def cancelled(self):
        """ Return True if the call was successfully cancelled. """
        with self._lock:
            return self._is_done and self._is_cancelled

    def result(self, timeout=None):
        """ Return the value returned by the call.

        If the call hasn’t yet completed then this method will wait up to
        timeout seconds. If the call hasn’t completed in timeout seconds, then
        a TimeoutError will be raised. timeout can be an int or float. If
        timeout is not specified or None, there is no limit to the wait time.

        If the future is cancelled before completing then CancelledError will
        be raised.

        If the call raised, this method will raise the same exception.
        """
        with self._lock:
            self._condition.wait(timeout)

            if not self._is_done:
                raise TimeoutError()
            elif self._is_cancelled:
                raise CancelledError()
            elif self._exception is not None:
                raise self._exception
            else:
                return self._result

    def exception(self, timeout=None):
        """ Return the exception raised by the call.

        If the call hasn’t yet completed then this method will wait up to
        timeout seconds. If the call hasn’t completed in timeout seconds, then
        a TimeoutError will be raised. timeout can be an int or float. If
        timeout is not specified or None, there is no limit to the wait time.

        If the future is cancelled before completing then CancelledError will
        be raised.

        If the call completed without raising, None is returned.
        """
        with self._lock:
            self._condition.wait(timeout)

            if not self._is_done:
                raise TimeoutError()
            elif self._is_cancelled:
                raise CancelledError()
            elif self._exception is not None:
                return self._exception
            else:
                return None

    def add_done_callback(self, fn):
        """ Attaches the callable fn to the future.

        fn will be called, with the future as its only argument, when the
        future is cancelled or finishes running. If fn was already added as a
        callback, this will raise an InternalError.

        Added callables are called in the order that they were added and are
        always called in a thread belonging to the process that added them. If
        the callable raises a Exception subclass, it will be logged and
        ignored. If the callable raises a BaseException subclass, the behavior
        is undefined.

        If the future has already completed or been cancelled, fn will be
        called immediately.
        """
        with self._lock:
            if self._is_done:
                if fn in self._callbacks:
                    raise InternalError('Callback is already registered.')

                self._callbacks.append(fn)
                do_call = False
            else:
                do_call = True

        if do_call:
            fn(self)

    def remove_done_callback(self, fn):
        """ Removes the callable fn to the future.

        If fn is not registered as a callback, this will raise an Exception.
        """
        with self._lock:
            try:
                self._callbacks.remove(fn)
            except ValueError:
                raise InternalError('Callback was not registered.')

    def set_result(self, result):
        """ Set the result of this Future. """
        self._result = result
        self._set_done()

    def set_cancel(self):
        """ Flag this Future as being cancelled. """
        self._is_cancelled = True
        self._set_done()

    def set_exception(self, exception):
        """ Indicates that an exception has occurred. """
        self._exception = exception
        self._set_done()

    def _set_done(self):
        """ Mark this future as done and return a callback function.
        """
        with self._lock:
            if self._is_done:
                raise InternalError('This future is already done.')

            self._is_done = True
            callbacks = list(self._callbacks)

            self._condition.notify_all()

        for callback_fn in callbacks:
            try:
                callback_fn(self)
            except Exception as e:
                self.logger.exception('Callback raised an exception.')


class TrajectoryFuture(object):
    logger = logging.getLogger('TrajectoryFuture')

    def __init__(self):
        from trajectory_msgs.msg import JointTrajectory
        from threading import Condition, Lock

        self._lock = Lock()
        self._handle = None

        # Flags:
        # - _cancelled: set if execution is cancelled (even if not by us)
        # - _done: set when execution terminates, regardless of the cause
        #
        # When _done is set to True, the _done_condition variable is notified
        # and the functions in _done_callbacks are called sequentially.
        self._cancelled = False
        self._done = False
        self._done_condition = Condition(self._lock)
        self._done_callbacks = []

        # Result variables:
        # - _result: set if execution succeeds
        # - _exception: set if an error occurrs.
        # 
        # Both of values remain None if execution is cancelled.
        self._result = None
        self._exception = None

        self._traj_actual = JointTrajectory()

    def cancel(self):
        with self._lock:
            if self._handle is None:
                raise InternalError('This TrajectoryFuture is not initialized.')
            elif self._cancelled:
                return True
            elif self._done:
                return False

            self._handle.cancel()
            return True

    def cancelled(self):
        with self._lock:
            return self._done and self._cancelled

    def running(self):
        with self._lock:
            return not self._done

    def done(self):
        with self._lock:
            return self._done

    def result(self, timeout=None):
        from concurrent.futures import CancelledError, TimeoutError

        with self._done_condition:
            condition_wait(self._done_condition, timeout, lambda: self._done)

            if not self._done:
                raise TimeoutError()
            elif self._cancelled:
                raise CancelledError()
            elif self._exception is not None:
                raise self._exception
            else:
                return self._result

    def partial_result(self):
        from copy import deepcopy

        with self._lock:
            return deepcopy(self._traj_actual)

    def exception(self, timeout=None):
        from concurrent.futures import CancelledError

        with self._done_condition:
            condition_wait(self._done_condition, timeout, lambda: self._done)

            if not self._done:
                raise TimeoutError()
            elif self._cancelled:
                raise CancelledError()
            elif self._exception is not None:
                return self._exception
            else:
                return None

    def add_done_callback(self, fn):
        with self._lock:
            if self._done:
                self._call_callback(fn)
            else:
                self._done_callbacks.append(fn)

    def _set_done(self, terminal_state, result):
        # NOTE: This function MUST be called with the lock acquired.

        from actionlib import TerminalState, get_name_of_constant
        from control_msgs.msg import FollowJointTrajectoryResult

        # The actionlib call succeeded, so "result" is valid.
        if terminal_state == TerminalState.SUCCEEDED:
            Result = FollowJointTrajectoryResult

            # Trajectory execution succeeded. Return the trajectory.
            if result.error_code == Result.SUCCESSFUL:
                self._result = self._traj_actual
            # Trajectory execution failed. Raise an exception.
            else:
                self._exception = TrajectoryExecutionFailed(
                    'Trajectory execution failed ({:s}): {:s}'.format(
                        get_name_of_constant(Result, result.error_code),
                        result.error_string))
        # Goal was cancelled. Note that this could have been one by another
        # thread or process, so _cancelled may be False.
        elif terminal_state not in [TerminalState.PREEMPTED,
                                    TerminalState.RECALLED]:
            self._cancelled = True
        else:
            self._exception = TrajectoryExecutionFailed(
                'Trajectory execution failed ({:s}): {:s}'.format(
                    get_name_of_constant(TerminalState, terminal_state),
                    self._handle.get_goal_status_text()))

        # Flag this future as "done".
        self._done = True
        self._done_condition.notify_all()

    def _call_callback(self, fn):
        try:
            fn(self._result)
        except Exception as e:
            self.logger.exception('Callback raised an exception.')

    def _transition_callback(self, handle):
        from actionlib import CommState

        state = handle.get_state()
        do_callbacks = False

        # Transition to the "done" state. This occurs when the trajectory
        # finishes for any reason (including an error).
        with self._lock:
            if not self._done and state == CommState.DONE:
                self._set_done(handle.get_terminal_state(),
                               handle.get_result())
                do_callbacks = True

        # Call any registered "done" callbacks. We intentionally do this
        # outside of _set_done so we can release the lock.
        if do_callbacks:
            for fn in self._done_callbacks:
                self._call_callback(fn)

    def _feedback_callback(self, feedback_msg):
        with self._lock:
            # Initialize the trajectory's start time with the timestamp of the
            # first observed feedback message.
            if not self._traj_actual.header.stamp:
                self._traj_actual.header.stamp = feedback_msg.header.stamp

            actual_waypoint = feedback_msg.actual
            actual_waypoint.time_from_start = feedback_msg.header.stamp \
                                            - self._traj_actual.header.stamp
            self._traj_actual.points.append(actual_waypoint)


class TrajectoryMode(ROSControlMode):
    def __init__(self, ns):
        from actionlib import ActionClient
        from control_msgs.msg import JointTrajectoryAction
        from threading import Lock

        self._lock = Lock()
        self._queue = []

        self._client = ActionClient(ns, JointTrajectoryAction)
        self._client.wait_for_server()

    def running(self):
        with self._lock:
            return bool(self._queue)

    def execute_ros_trajectory(self, traj_msg):
        from control_msgs.msg import JointTrajectoryActionGoal

        goal_msg = JointTrajectoryActionGoal(
            trajectory=traj_msg,
            path_tolerance=[], # use default values
            goal_tolerance=[], # use default values
            goal_time_tolerance=0. # use default value
        )

        # Return a TrajectoryFuture to track execution state.
        traj_future = TrajectoryFuture()
        traj_future._handle = self._client.send_goal(
            goal_msg,
            transition_cb=traj_future._transition_callback,
            feedback_cb=traj_future._feedback_callback
        )

        # Add this trajectory to the queue of running trajectories. Remove it
        # when it finishes executing.
        with self._lock:
            self._queue.append(traj_future)

        def remove_from_queue(_):
            with self._lock:
                self._queue.remove(traj_future)

        traj_future.add_done_callback(remove_from_queue)

        return traj_future
