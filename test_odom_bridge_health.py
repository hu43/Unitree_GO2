"""Unit tests for the sport odometry relay health predicate."""

from odom_bridge_health import is_odom_bridge_healthy


def test_requires_process_and_ros_publisher():
    assert not is_odom_bridge_healthy(False, 1, 100.0, now=101.0)
    assert not is_odom_bridge_healthy(True, 0, 100.0, now=101.0)


def test_rejects_missing_or_stale_odom():
    assert not is_odom_bridge_healthy(True, 1, 0.0, now=101.0)
    assert not is_odom_bridge_healthy(True, 1, 98.9, now=101.0)
    assert is_odom_bridge_healthy(True, 1, 99.5, now=101.0)


def test_rejects_clock_skew():
    assert not is_odom_bridge_healthy(True, 1, 101.1, now=101.0)
