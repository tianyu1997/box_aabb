#!/usr/bin/env bash
set -euo pipefail

LOG="${HOME}/safe_reboot_fix_$(date +%F_%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1

echo "[1/4] Reinstall NVIDIA packages"
apt-get update
apt-get install --reinstall -y \
  nvidia-driver-590 \
  nvidia-dkms-590 \
  nvidia-kernel-common-590 \
  nvidia-kernel-source-590

echo "[2/4] Rebuild initramfs"
update-initramfs -u

echo "[3/4] Post-check"
nvidia-smi || true
systemctl --failed || true

echo "[4/4] Done. Log: $LOG"
echo "Now reboot and verify with: journalctl -b -1 -n 120 --no-pager"
