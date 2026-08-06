#!/usr/bin/env bash
#
# install-docker.sh — install Docker CE + compose plugin on Ubuntu 20.04 (focal).
#
# For the Tier 2 attacker container (see attacker/README.md). Installs from
# Docker's OFFICIAL apt repo so you get current Docker Engine and the
# `docker compose` v2 plugin the attacker/ compose file needs.
#
# Run it with sudo:   sudo ./install-docker.sh
# It re-runs safely (idempotent): re-adding the repo/key and re-installing are no-ops.
#
set -euo pipefail

# --- must be root (the whole point is the privileged steps) ---------------
if [[ "${EUID}" -ne 0 ]]; then
  echo "This script installs system packages and must run as root." >&2
  echo "Re-run:  sudo $0" >&2
  exit 1
fi

# --- the non-root user to add to the docker group -------------------------
# When invoked via sudo, $SUDO_USER is the human who called it. Fall back to
# logname if somehow unset. We add THAT user (not root) to the docker group.
TARGET_USER="${SUDO_USER:-$(logname 2>/dev/null || true)}"

# --- sanity: this script targets Ubuntu focal -----------------------------
. /etc/os-release 2>/dev/null || true
if [[ "${VERSION_CODENAME:-}" != "focal" ]]; then
  echo "[warn] Expected Ubuntu 20.04 (focal); found '${VERSION_CODENAME:-unknown}'." >&2
  echo "[warn] Continuing, but the apt repo line below is pinned to focal." >&2
fi

echo "[1/5] Removing any old/conflicting Docker packages (safe if none present)..."
apt-get remove -y docker docker-engine docker.io containerd runc 2>/dev/null || true

echo "[2/5] Installing prerequisites and Docker's official GPG key + repo..."
apt-get update
apt-get install -y ca-certificates curl gnupg
install -m 0755 -d /etc/apt/keyrings
# --dearmor is not idempotent (gpg refuses to overwrite); remove first.
rm -f /etc/apt/keyrings/docker.gpg
curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu focal stable" \
  > /etc/apt/sources.list.d/docker.list

echo "[3/5] Installing Docker Engine, CLI, containerd, buildx, and compose plugin..."
apt-get update
apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

echo "[4/5] Enabling and starting the Docker daemon..."
systemctl enable --now docker

if [[ -n "${TARGET_USER}" && "${TARGET_USER}" != "root" ]]; then
  echo "[5/5] Adding user '${TARGET_USER}' to the 'docker' group..."
  usermod -aG docker "${TARGET_USER}"
  GROUP_NOTE="  Group change for '${TARGET_USER}' takes effect in a NEW login session.
  Log out and back in, or run:   newgrp docker
  Until then, docker commands need sudo."
else
  echo "[5/5] Skipping docker-group add (no non-root target user detected)."
  GROUP_NOTE="  No non-root user was added to the docker group; add one with:
    sudo usermod -aG <you> docker   then re-login."
fi

echo
echo "=== Installed versions ==="
docker --version
docker compose version

echo
echo "Docker CE installed and the daemon is running."
echo "${GROUP_NOTE}"
echo
echo "Verify (after re-login / newgrp docker):"
echo "  docker run --rm hello-world"
echo
echo "Then build the attacker container:"
echo "  cd attacker && docker compose build"
