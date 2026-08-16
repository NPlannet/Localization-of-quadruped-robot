# Raspberry Pi Host Setup

These settings live on the Raspberry Pi host and are not preserved in a Docker
image export.

## XGO UART

Add to `/boot/firmware/config.txt`:

```ini
enable_uart=1
dtoverlay=miniuart-bt
```

Remove `console=serial0,115200` from `/boot/firmware/cmdline.txt`, then disable
serial consoles:

```bash
sudo systemctl disable --now serial-getty@ttyAMA0.service
sudo systemctl disable --now serial-getty@ttyS0.service
sudo usermod -aG dialout,tty "$USER"
```

Create `/etc/udev/rules.d/99-xgo-serial.rules`:

```udev
KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"
KERNEL=="ttyS0", GROUP="dialout", MODE="0660"
KERNEL=="ttyUSB[0-9]*", GROUP="dialout", MODE="0660"
```

Reload the rules and reboot:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo reboot
```

Only one process or container may open `/dev/ttyAMA0`.

## Camera and LiDAR

The LD19 normally appears as `/dev/ttyUSB0`. Verify camera devices with:

```bash
rpicam-hello --list-cameras
ls -l /dev/media* /dev/video* /dev/vchiq
```

The robot container runs privileged so libcamera can access the complete
Raspberry Pi media graph.

## Reproducibility checks

- Use the same image tag, Git commit, ROS distribution, and `ROS_DOMAIN_ID`.
- Synchronize the Pi clock; incorrect timestamps break TF and sensor fusion.
- Use an adequate power supply and check `vcgencmd get_throttled` after
  intermittent USB or camera failures.
- Keep the Foxglove bridge restricted to the intended robot and operator.

A container export does not preserve boot configuration, systemd services,
udev rules, host groups, Wi-Fi, device identities, or sensor calibration.
