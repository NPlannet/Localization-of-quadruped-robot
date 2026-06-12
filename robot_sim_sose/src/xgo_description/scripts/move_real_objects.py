import math
import subprocess
import time


WORLD = "real_objects_world"
INTERVAL = 0.05
CHAIR_PERIOD = 36.0
PERSON_PERIOD = 27.0
MIN_DIRECTION_SPEED = 1e-4
YAW_TIME_CONSTANT = 0.35


def smooth_oscillation(start, end, period, now):
    phase = (2.0 * math.pi * now) / period
    progress = 0.5 - 0.5 * math.cos(phase)
    position = start + (end - start) * progress
    velocity = (end - start) * (math.pi / period) * math.sin(phase)
    return position, velocity


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def smooth_angle(current, target, dt):
    alpha = 1.0 - math.exp(-dt / YAW_TIME_CONSTANT)
    angle_error = wrap_angle(target - current)
    return wrap_angle(current + alpha * angle_error)


def choose_target_yaw(current_yaw, velocity, travel_direction, forward_yaw, backward_yaw):
    if abs(velocity) < MIN_DIRECTION_SPEED:
        return current_yaw

    moving_toward_end = velocity * travel_direction > 0.0
    return forward_yaw if moving_toward_end else backward_yaw


def set_pose(name, x, y, z, yaw):
    half_yaw = yaw * 0.5
    request = (
        f'name: "{name}" '
        f"position {{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }} "
        f"orientation {{ z: {math.sin(half_yaw):.6f} w: {math.cos(half_yaw):.6f} }}"
    )
    subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{WORLD}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "300",
            "--req",
            request,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def main():
    start_time = time.monotonic()
    last_update = start_time
    current_yaws = {
        "moving_chair": -math.pi / 2,
        "walking_person": 0.0,
    }

    while True:
        now = time.monotonic()
        elapsed = now - start_time
        dt = max(now - last_update, 1e-3)
        last_update = now

        chair_y, chair_velocity = smooth_oscillation(-1.7, -4.7, CHAIR_PERIOD, elapsed)
        chair_target_yaw = choose_target_yaw(
            current_yaws["moving_chair"],
            chair_velocity,
            -4.7 - (-1.7),
            -math.pi / 2,
            math.pi / 2,
        )
        current_yaws["moving_chair"] = smooth_angle(
            current_yaws["moving_chair"],
            chair_target_yaw,
            dt,
        )
        set_pose("moving_chair", 1.2, chair_y, 0.45, current_yaws["moving_chair"])

        person_x, person_velocity = smooth_oscillation(-2.8, -0.8, PERSON_PERIOD, elapsed)
        person_target_yaw = choose_target_yaw(
            current_yaws["walking_person"],
            person_velocity,
            -0.8 - (-2.8),
            0.0,
            math.pi,
        )
        current_yaws["walking_person"] = smooth_angle(
            current_yaws["walking_person"],
            person_target_yaw,
            dt,
        )
        set_pose("walking_person", person_x, -2.0, 0.45, current_yaws["walking_person"])

        time.sleep(INTERVAL)


if __name__ == "__main__":
    main()
