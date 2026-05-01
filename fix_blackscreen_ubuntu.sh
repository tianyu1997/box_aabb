#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run as root: sudo bash $0"
  exit 1
fi

TARGET_DRIVER="${TARGET_DRIVER:-570}"
DISABLE_SLEEP="${DISABLE_SLEEP:-1}"
LOG="${HOME:-/root}/fix_blackscreen_$(date +%F_%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1

echo "[info] Starting black-screen repair"
echo "[info] Target driver branch: ${TARGET_DRIVER}"
echo "[info] Log file: ${LOG}"

apt_pkg_exists() {
  local pkg="$1"
  apt-cache policy "$pkg" 2>/dev/null | grep -q "Candidate:"
}

install_if_available() {
  local pkg="$1"
  if apt_pkg_exists "$pkg"; then
    echo "[info] Installing/reinstalling ${pkg}"
    apt-get install --reinstall -y "$pkg"
  else
    echo "[warn] Package not found: ${pkg}"
  fi
}

echo "[1/7] Update apt metadata"
apt-get update

echo "[2/7] Reinstall NVIDIA stack"
install_if_available "nvidia-driver-${TARGET_DRIVER}"
install_if_available "nvidia-dkms-${TARGET_DRIVER}"
install_if_available "nvidia-kernel-common-${TARGET_DRIVER}"
install_if_available "nvidia-kernel-source-${TARGET_DRIVER}"
install_if_available "nvidia-utils-${TARGET_DRIVER}"

echo "[3/7] Force GDM to Xorg (disable Wayland)"
if [[ -f /etc/gdm3/custom.conf ]]; then
  cp /etc/gdm3/custom.conf /etc/gdm3/custom.conf.bak.$(date +%s)
  if grep -q '^#\?WaylandEnable=' /etc/gdm3/custom.conf; then
    sed -i 's/^#\?WaylandEnable=.*/WaylandEnable=false/' /etc/gdm3/custom.conf
  else
    awk '
      BEGIN { done=0 }
      /^\[daemon\]$/ { print; print "WaylandEnable=false"; done=1; next }
      { print }
      END { if (!done) { print "[daemon]"; print "WaylandEnable=false" } }
    ' /etc/gdm3/custom.conf > /tmp/custom.conf.$$ && mv /tmp/custom.conf.$$ /etc/gdm3/custom.conf
  fi
else
  cat >/etc/gdm3/custom.conf <<'EOF'
[daemon]
WaylandEnable=false
EOF
fi

echo "[4/7] Configure NVIDIA kernel params"
cat >/etc/modprobe.d/nvidia-blackscreen-fix.conf <<'EOF'
options nvidia-drm modeset=1
options nvidia NVreg_PreserveVideoMemoryAllocations=1
EOF

echo "[5/7] Update GRUB kernel cmdline"
if [[ -f /etc/default/grub ]]; then
  cp /etc/default/grub /etc/default/grub.bak.$(date +%s)
  if grep -q '^GRUB_CMDLINE_LINUX_DEFAULT=' /etc/default/grub; then
    line="$(grep '^GRUB_CMDLINE_LINUX_DEFAULT=' /etc/default/grub)"
    current="${line#*=}"
    current="${current%\"}"
    current="${current#\"}"
    for arg in nvidia-drm.modeset=1 nvidia.NVreg_PreserveVideoMemoryAllocations=1; do
      if [[ " ${current} " != *" ${arg} "* ]]; then
        current="${current} ${arg}"
      fi
    done
    current="$(echo "$current" | xargs)"
    sed -i "s|^GRUB_CMDLINE_LINUX_DEFAULT=.*|GRUB_CMDLINE_LINUX_DEFAULT=\"${current}\"|" /etc/default/grub
  else
    echo 'GRUB_CMDLINE_LINUX_DEFAULT="quiet splash nvidia-drm.modeset=1 nvidia.NVreg_PreserveVideoMemoryAllocations=1"' >> /etc/default/grub
  fi
fi

echo "[6/7] Rebuild initramfs and grub"
update-initramfs -u
if command -v update-grub >/dev/null 2>&1; then
  update-grub
fi

echo "[7/7] Optional: disable suspend/hibernate targets"
if [[ "${DISABLE_SLEEP}" == "1" ]]; then
  systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target || true
fi

echo "[check] nvidia-smi"
nvidia-smi || true

echo "[done] Black-screen repair finished"
echo "[done] Reboot now: reboot"
echo "[done] After reboot check logs: journalctl -b -p err --no-pager | tail -n 200"
