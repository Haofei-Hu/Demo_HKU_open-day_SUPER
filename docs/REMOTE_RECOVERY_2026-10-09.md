# 10 月 9 日：AX900 无线网卡、NoMachine 与重启恢复

## 已记录状态

- Ubuntu 22.04，当前内核 `6.8.0-138-generic`；机载电脑 Intel i915，显示接口 `DP-1`。
- 网卡为 UGREEN **AX900／AIC8800D80**，使用 `UGREEN_AIC-AX900_LinuxDriver_V1.1.zip`。AX300 驱动包只有 DC 固件，不能用它提供 D80 固件。
- Wi-Fi 接口 `wlx6c1ff7880bdf`，NetworkManager 连接名称最终为 `Mars_Office 1`，最后确认的地址 `192.168.6.146/24`。
- MID360 专用网卡 `enp4s0`，电脑地址 `192.168.50.125/24`。无线网络不占用雷达以太网。
- NoMachine **9.8.2**，状态显示 nxserver／nxnode／nxd 已启用；Mac 远程桌面已恢复可操作。
- Wi-Fi `connection.autoconnect=yes`、`connection.permissions` 为空、`psk-flags=0`，连接密码已由本机 NetworkManager 保存，不放进仓库。

原来安装的 `nomachine-personal-edition 10.2.3-1` 报 `No subscription found on this server`，属于该已安装版本的订阅问题。更换到 9.8.2 后，`nxserver --version` 显示 `NoMachine - Version 9.8.2`，不再显示订阅报错；这不是更改网络 IP 或猜测密码解决的。

## 每次开机先做的检查

以下运行在机载 Ubuntu，Mac 的终端不直接运行 ROS／Linux显示诊断命令：

```bash
nmcli device status
ip -4 -br addr show dev wlx6c1ff7880bdf
sudo /usr/NX/bin/nxserver --status
systemctl is-enabled ugreen-ax900.service
```

无线为 connected 且拿到地址后，Mac 的 NoMachine 手动连接：NX 协议、当前无线 IP、端口 4000、本机 Ubuntu 用户 `mars` 及其登录密码。IP 可能变化，不能只根据旧的 `192.168.6.146` 判断服务是否坏了。

## AX900 识别与开机初始化

设备会经历三种 USB 身份：

| USB ID | 状态 |
| --- | --- |
| `a69c:5723` | Aic MSC，安装器／存储模式 |
| `a69c:8d80` | 固件加载阶段 |
| `368b:8d85` | AIC8800D80 运行阶段 |

运行阶段的 `lsusb -t` 已显示 `Driver=aic8800_fdrv`，之后出现无线接口。看到固件加载后的短暂 USB 断开／重新枚举不等于驱动失败。

本次问题与修复经过：

1. 编译报 `/bin/sh: gcc-12: not found`。内核头文件已存在，缺的是内核使用的 GCC 12 编译器。
2. `make install` 写 `/lib/modules/...` 报权限不足，安装阶段需要 sudo。
3. 初始用错 AX300 包，`/lib/firmware/aic8800D80/fw_patch_table_8800d80_u02.bin` 缺失；改用 AX900 包和匹配固件后加载成功。
4. 开机设备仍停在 MSC。早期 udev 直接运行 `usb_modeswitch` 以及立即切换的服务出现超时 `-7`，接口没有生成。
5. 归档的服务由 systemd 执行切换，先等待 10 秒，再以 20 秒 timeout 执行 EJECT；服务整体超时 90 秒，保留有限重试。NetworkManager 在接口生成后按已保存连接自动连接。

仓库 `system/` 中保存了最终建议的启动脚本、服务、udev 规则与模块加载列表。已记录手动恢复成功、无线自动连接设置，以及接口已经存在时服务打印 `AX900 Wi-Fi interface ready`。**最后的延迟切换方案尚未收到一次完整冷启动的成功回传**，不能把该记录写成已验证每次开机都会恢复。

### 恢复系统配置

只有这台机载电脑已安装对应内核的 AX900 模块及 D80 固件、且要恢复这些配置时，才在仓库根目录运行；正常连通时无需重复安装。飞机保持地面、未解锁，重启网络服务可能断开远程桌面。

```bash
modinfo aic_load_fw
modinfo aic8800_fdrv
command -v usb_modeswitch
```

确认上述条件后，备份已有文件再安装：

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

这套配置不包含厂商驱动源码、固件或 Wi-Fi 密码。Linux 内核升级后需要重新确认模块适配，不能只复制旧内核的 `.ko`。

### 临时恢复与检查

```bash
sudo systemctl restart ugreen-ax900.service
sudo journalctl -b -u ugreen-ax900.service --no-pager -n 35
nmcli radio wifi on
nmcli device status
```

服务可能需要数十秒等待 USB 切换与固件初始化。如果无线出现但仍在 `getting IP configuration`，等 DHCP 完成后再检查；该状态还没有可用的 IPv4 地址。

已有接口但未连接时：

```bash
nmcli connection up 'Mars_Office 1'
ip -4 -br addr show dev wlx6c1ff7880bdf
```

只有还没有保存连接时才执行交互连接，不把密码写进命令：

```bash
nmcli --ask device wifi connect 'Mars_Office' ifname wlx6c1ff7880bdf
```

## 无显示器桌面

机载电脑使用 GNOME／Wayland、Intel i915。此前出现过 NoMachine 的其他桌面会话与物理桌面不同、应用打开到另一个桌面，以及拔掉显示器后画面冻结／白屏。

本次通过实际显示器的 EDID 覆盖及强制 DP-1 启用，用户已回传“能操作了”。本机持久化方案使用以下 GRUB 参数：

```text
drm.edid_firmware=DP-1:edid/nomachine-dp1.bin video=DP-1:e
```

EDID 文件位于本机 `/lib/firmware/edid/nomachine-dp1.bin`，通过 initramfs hook 加入启动镜像。仓库不生成或分发别的显示器 EDID，不提供会覆盖本机 GRUB 的通用一键脚本。持久化配置做过语法检查；无显示器冷启动的最终可操作性还需实测。

故障时查看物理会话、显示接口和服务状态：

```bash
sudo /usr/NX/bin/nxserver --list
sudo /usr/NX/bin/nxserver --status
for display_status in /sys/class/drm/card*-*/status; do
  printf '%s: ' "$(basename "$(dirname "$display_status")")"
  cat "$display_status"
done
```

远程桌面恢复后，按 [完整测试工作流](FIRST_HOVER_2026-10-09.md) 恢复 ROS 终端；不要在飞行中重启无线、定位或会话初始化节点。
