import threading
import traceback

class Thread(threading.Thread):
    """A thread that can be cleanly stopped on demand."""

    def __init__(self, target=None, args=(), kwargs=None, interval=0.5, daemon=True):
        """
        target   : the function to run repeatedly (receives stop_event as last check point)
        args     : positional args passed to target
        kwargs   : keyword args passed to target
        interval : how often (seconds) the loop checks the stop flag
        daemon   : whether the thread dies automatically if main program exits
        """
        super().__init__(daemon=daemon)
        self._target_func = target
        self._args = args
        self._kwargs = kwargs or {}
        self._interval = interval
        self._stop_event = threading.Event()

    def run(self):
        while not self._stop_event.is_set():
            if self._target_func:
                try:
                    self._target_func(*self._args, **self._kwargs)
                except Exception:
                    # Without this, a target that raises dies silently --
                    # the thread just vanishes with no log at all, and
                    # whatever's tracking it (e.g. Swarm.current_function)
                    # is left thinking a worker is still running when it
                    # isn't. Stop rather than let the outer while loop
                    # immediately re-invoke a target that just raised --
                    # that would spin into a tight crash loop instead.
                    print(f"[{self.name}] target raised an unhandled exception -- stopping:")
                    traceback.print_exc()
                    self._stop_event.set()
                    break
            self._stop_event.wait(self._interval)  # sleeps but wakes early if stopped
        print(f"[{self.name}] stopped cleanly.")

    def stop(self,call_back_action=None):
        """Signal the thread to stop."""
        if call_back_action is not None:
            print("call_back_action is available")
            call_back_action()
        self._stop_event.set()

    def stopped(self):
        """Check if a stop has been requested."""
        return self._stop_event.is_set()