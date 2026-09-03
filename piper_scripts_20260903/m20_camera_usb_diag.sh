#!/usr/bin/env bash
set -Eeuo pipefail
export LC_ALL=C

echo "=== ROOT USB DIAGNOSTIC ==="
date -Is
id

device_line="$(lsusb -d 8086:0b3a | head -n1)"
echo "$device_line"
bus="$(awk '{print $2}' <<<"$device_line")"
device="$(awk '{gsub(":", "", $4); print $4}' <<<"$device_line")"
usb_node="/dev/bus/usb/$bus/$device"

echo
echo "=== USB NODE BEFORE UDEV REFRESH ==="
ls -l "$usb_node"
udevadm info --query=property --name="$usb_node" | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_USB_INTERFACES|DEVNAME|DEVTYPE)=' || true

echo
echo "=== VIDEO/HID NODES ==="
ls -l /dev/video* /dev/hidraw* 2>/dev/null || true

echo
echo "=== REFRESH UDEV RULES ==="
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=usb
udevadm settle --timeout=10
sleep 2
ls -l "$usb_node"

echo
echo "=== ROOT REALSENSE ENUMERATION ==="
set +u
source /opt/ros/foxy/setup.bash
set -u
rs-enumerate-devices -s || true

echo
echo "=== VIDEO INTERFACE DETAILS ==="
for node in /sys/class/video4linux/video*; do
  printf '%s: ' "$(basename "$node")"
  cat "$node/name" 2>/dev/null || true
  readlink -f "$node/device" || true
done

echo
echo "=== USB DESCRIPTOR SUMMARY ==="
lsusb -v -d 8086:0b3a 2>/dev/null | \
  grep -E '(^Bus|bcdUSB|bDeviceClass|bInterfaceClass|bInterfaceSubClass|bInterfaceProtocol|iProduct|MaxPower)' || true

echo
echo "=== KERNEL LOG (REALSENSE/UVC/USB) ==="
dmesg | grep -Ei 'realsense|uvc|usb 5-|usb 6-|xhci' | tail -n 120 || true
