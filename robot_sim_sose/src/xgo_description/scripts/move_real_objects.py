import math
import subprocess
import time


WORLD = "real_objects_world"
INTERVAL = 0.2


def triangle(start, end, period, now):
    phase = (now % period) / period
    if phase < 0.5:
        ratio = phase * 2.0
        forward = True
    else:
        ratio = (1.0 - phase) * 2.0
        forward = False
    return start + (end - start) * ratio, forward


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
    while True:
        elapsed = time.monotonic() - start_time

        chair_y, chair_forward = triangle(-1.7, -4.7, 12.0, elapsed)
        chair_yaw = -math.pi / 2 if chair_forward else math.pi / 2
        set_pose("moving_chair", 1.2, chair_y, 0.45, chair_yaw)

        person_x, person_forward = triangle(-2.8, -0.8, 9.0, elapsed)
        person_yaw = 0.0 if person_forward else math.pi
        set_pose("walking_person", person_x, -2.0, 0.45, person_yaw)

        time.sleep(INTERVAL)


if __name__ == "__main__":
    main()
