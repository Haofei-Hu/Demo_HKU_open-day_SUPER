# AX900, NoMachine, and headless recovery

## Current display arrangement — October 12

The aircraft now uses a physical HDMI display emulator. `HDMI-A-1` was detected as connected. The software DP-1 override and initramfs hook were backed up and removed, and the rebooted kernel command line no longer contains the EDID/forced-output parameters. NoMachine remains the desktop access method. Remote resolution can be adjusted through Ubuntu Displays and client scaling.

The EDID procedure below is the historical software setup, retained for reference rather than required for the current HDMI emulator.

## Deployed environment

| Component | Recorded state |
| --- | --- |
| OS/kernel | Ubuntu 22.04, `6.8.0-138-generic` |
| Graphics | Intel i915, physical output `DP-1` |
| USB Wi-Fi | UGREEN AX900 / AIC8800D80, AX900 LinuxDriver V1.1 |
| Wi-Fi interface | `wlx6c1ff7880bdf` |
| NetworkManager profile | `Mars_Office 1` |
| Last Wi-Fi address | `192.168.6.146/24` |
| MID360 Ethernet | `enp4s0`, onboard `192.168.50.125/24` |
| Remote desktop | NoMachine 9.8.2; Mac access to onboard Ubuntu desktop operational |

The earlier Personal Edition 10.2.3 installation returned `No subscription found on this server`. Replacing it with NoMachine 9.8.2 restored the local server and desktop connection. Network credentials are stored locally in NetworkManager, not in the repository.

## Startup checks

On onboard Ubuntu:

```bash
nmcli device status
ip -4 -br addr show dev wlx6c1ff7880bdf
sudo /usr/NX/bin/nxserver --status
systemctl is-enabled ugreen-ax900.service
```

Connect using NX, the current Wi-Fi IP, port 4000, and the local Ubuntu username/password. Recheck DHCP addresses after reboot.

From a Mac on the same network:

```bash
ping -c 3 192.168.6.146
nc -vz -G 3 192.168.6.146 4000
```

## AX900 device lifecycle

| USB ID | Stage |
| --- | --- |
| `a69c:5723` | Installer/storage mode: Aic MSC |
| `a69c:8d80` | Firmware loader |
| `368b:8d85` | Running AIC8800D80 |

Firmware upload can cause a normal USB disconnect/re-enumeration. The running interface uses `aic8800_fdrv`.

Resolved setup issues:

- Kernel headers existed, but compilation initially lacked `gcc-12`; the kernel compiler version was GCC 12.
- Module installation under `/lib/modules` required elevated privileges.
- The AX300 package contained DC firmware rather than the required D80 firmware. The AX900 package supplied the matching files.
- Immediate EJECT operations during startup timed out. The archived systemd helper waits 10 seconds, uses a bounded 20-second modeswitch call, and limits retries within a 90-second service timeout.
- Saved NetworkManager autoconnect starts after the Wi-Fi interface appears.

Manual recovery, driver binding, Wi-Fi association, and remote desktop operation have succeeded. Repeated cold-boot restoration with the final delayed modeswitch setup remains a separate check.

## Restore startup configuration

The files under [system/](../system/) assume compatible AX900 modules and D80 firmware are already installed. Module compatibility must be rechecked after a kernel upgrade.

Verify prerequisites:

```bash
modinfo aic_load_fw
modinfo aic8800_fdrv
command -v usb_modeswitch
```

From the repository root, back up existing configuration before installing the startup files. Do this grounded and disarmed; network restarts can disconnect remote access.

```bash
ax900_backup_dir=$(mktemp -d "$HOME/Downloads/ax900_config_backup_XXXXXX")
for config_file in /usr/local/sbin/ugreen-ax900-start /etc/systemd/system/ugreen-ax900.service /etc/udev/rules.d/99-ugreen-ax900.rules /etc/modules-load.d/ugreen-ax900.conf; do
  if [ -f "$config_file" ]; then
    sudo cp --parents "$config_file" "$ax900_backup_dir/"
  fi
done
sudo install -m 755 system/ugreen-ax900-start /usr/local/sbin/ugreen-ax900-start
sudo install -m 644 system/ugreen-ax900.service /etc/systemd/system/ugreen-ax900.service
sudo install -m 644 system/99-ugreen-ax900.rules /etc/udev/rules.d/99-ugreen-ax900.rules
sudo install -m 644 system/ugreen-ax900.conf /etc/modules-load.d/ugreen-ax900.conf
sudo systemctl daemon-reload
sudo udevadm control --reload-rules
sudo nmcli connection modify 'Mars_Office 1' connection.autoconnect yes connection.permissions ''
sudo systemctl enable ugreen-ax900.service
```

The repository does not distribute vendor firmware, kernel modules, or Wi-Fi credentials.

## Temporary recovery

```bash
sudo systemctl restart ugreen-ax900.service
sudo journalctl -b -u ugreen-ax900.service --no-pager -n 35
nmcli radio wifi on
nmcli device status
```

USB and firmware initialization can take tens of seconds. `Getting IP configuration` means DHCP has not yet completed. If the interface exists but is not connected:

```bash
nmcli connection up 'Mars_Office 1'
ip -4 -br addr show dev wlx6c1ff7880bdf
```

For a first connection without saved credentials:

```bash
nmcli --ask device wifi connect 'Mars_Office' ifname wlx6c1ff7880bdf
```

## Historical software headless desktop

The onboard GNOME/Wayland desktop initially showed mismatched sessions, applications opening on another display, and freezing/blank screens after monitor removal. The recorded physical-display EDID override and forced DP-1 output restored usable headless operation.

The machine-specific GRUB parameters are:

```text
drm.edid_firmware=DP-1:edid/nomachine-dp1.bin video=DP-1:e
```

The local EDID file `/lib/firmware/edid/nomachine-dp1.bin` is included in the initramfs. This repository does not supply a generic EDID or a script that overwrites GRUB. Persistent configuration syntax was checked; repeated headless cold boots remain on the recovery validation plan.

Inspect sessions and output state when diagnosing display issues:

```bash
sudo /usr/NX/bin/nxserver --list
sudo /usr/NX/bin/nxserver --status
for display_status in /sys/class/drm/card*-*/status; do
  printf '%s: ' "$(basename "$(dirname "$display_status")")"
  cat "$display_status"
done
```

After network/display recovery, restore the ROS stack using the [flight workflow](FIRST_HOVER_2026-10-09.md). Network and localization restarts belong to grounded maintenance, not an active flight.
