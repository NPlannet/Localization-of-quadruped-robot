# Raspberry Pi Host Configuration

These are host settings: they are not stored inside an exported Docker image.
Apply and verify them on every XGO Raspberry Pi before debugging ROS nodes.

## UART for the XGO controller

In `/boot/firmware/config.txt`, enable the UART and move Bluetooth away from the
primary PL011 interface:

```ini
enable_uart=1
dtoverlay=miniuart-bt
```

Remove `console=serial0,115200` from `/boot/firmware/cmdline.txt`. Keep that file
on one line. Disable serial getty services that could own the controller port:

```bash
sudo systemctl disable --now serial-getty@ttyAMA0.service
sudo systemctl disable --now serial-getty@ttyS0.service
```

Add the login user to the serial groups:

```bash
sudo usermod -aG dialout,tty "$USER"
```

Create `/etc/udev/rules.d/99-xgo-serial.rules`:

```udev
KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"
KERNEL=="ttyS0", GROUP="dialout", MODE="0660"
KERNEL=="ttyUSB[0-9]*", GROUP="dialout", MODE="0660"
```

Then reload the rules and reboot:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo reboot
```

After reboot, verify that `/dev/ttyAMA0` exists and is not held by another
process. Only one program or container may open the XGO controller port.

## Camera and LiDAR devices

The camera container needs the Raspberry Pi media/video devices that exist on
that specific host, typically `/dev/media0`, `/dev/media1`, `/dev/media2`,
`/dev/video*`, and `/dev/vchiq`. The robot Compose file uses privileged mode so
libcamera can discover the complete pipeline. Verify it on the host with:

```bash
rpicam-hello --list-cameras
ls -l /dev/media* /dev/video* /dev/vchiq
```

The LD19 normally appears as `/dev/ttyUSB0`. Device numbering can change when
USB devices are reconnected; a stable udev symlink is preferable for a fleet.

## Docker, networking, and time

- Install Docker Engine and the Compose plugin on the host.
- Keep the same ROS 2 distribution, dependency versions, image tag, and Git
  commit on every robot.
- Use one deliberate `ROS_DOMAIN_ID` across the robot and operator laptop.
- Ensure multicast/firewall rules do not expose DDS control topics to unrelated
  robots. The Foxglove bridge is configured with a topic allowlist.
- Keep the Pi clock synchronized. Bad timestamps cause TF and sensor-message
  filter failures.
- Use an adequate power supply; undervoltage can destabilize USB and camera
  devices. Check `vcgencmd get_throttled` when hardware behaves intermittently.

## What an image export does not preserve

A committed/exported container preserves installed packages and files inside
that container. It does not preserve firmware configuration, boot arguments,
systemd state, udev rules, host groups, device identity, Wi-Fi, or physical
sensor calibration. Record those host settings separately for reproducible
deployment.
