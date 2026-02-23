import sys
import utime

CRASH_LOG_PATH = "/crash.log"


def write_crash_log(exc):
    """Write exception traceback to onboard flash. Overwrites any previous log.

    Uses only onboard resources (flash filesystem + utime).
    Never raises — crash handler must be bulletproof.
    """
    try:
        with open(CRASH_LOG_PATH, "w") as f:
            f.write("ticks_ms: {}\n".format(utime.ticks_ms()))
            sys.print_exception(exc, f)
    except Exception:
        pass