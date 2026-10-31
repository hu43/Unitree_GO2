"""Small, dependency-free health checks for the sport odometry relay."""

import time


def is_odom_bridge_healthy(process_alive, publisher_count, odom_stamp,
                           now=None, max_age=2.0):
    """Return whether the relay is alive, discoverable, and publishing fresh data."""
    if not process_alive or publisher_count < 1 or odom_stamp <= 0.0:
        return False
    now = time.time() if now is None else float(now)
    age = now - float(odom_stamp)
    return 0.0 <= age < max_age
