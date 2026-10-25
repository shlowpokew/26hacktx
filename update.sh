#!/usr/bin/env bash
# Pull the latest pushed commit and apply it to the running stack.
# ros2-update.timer runs this every minute; it is also safe to run by hand.
# Usage: bash update.sh [--force]   (--force rebuilds/applies even with no new commit)
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec 9<update.sh
flock -n 9 || { echo 'Another update is already running.'; exit 0; }

# Git runs as the checkout's owner (their SSH keys, no root-owned files);
# Docker runs as root.
if [[ $EUID -eq 0 ]]; then
  owner=$(stat -c %U .)
  as_owner() { sudo -H -u "$owner" "$@"; }
  as_root() { "$@"; }
else
  as_owner() { "$@"; }
  as_root() { sudo "$@"; }
fi

force=false
[[ ${1:-} == --force ]] && force=true

if ! as_owner git rev-parse --verify --quiet '@{u}' >/dev/null; then
  echo 'This branch has no upstream. Run: git branch --set-upstream-to=origin/main' >&2
  exit 1
fi
as_owner git fetch --quiet
old=$(as_owner git rev-parse HEAD)
new=$(as_owner git rev-parse '@{u}')
if [[ $old == "$new" ]] && ! $force; then
  exit 0
fi

changed=''
if [[ $old != "$new" ]]; then
  if ! as_owner git merge --ff-only --quiet '@{u}'; then
    echo 'Cannot fast-forward: commit or discard local edits on the robot, then retry.' >&2
    exit 1
  fi
  changed=$(as_owner git diff --name-only "$old" "$new")
  echo "Updated ${old:0:7} -> ${new:0:7}:"
  sed 's/^/  /' <<<"$changed"
fi

# A deliberately stopped service stays stopped; just prepare the new image.
if [[ -n $(as_root docker compose ps -aq --status exited mapping) ]]; then
  as_root docker compose build
  echo 'mapping is stopped; changes apply on the next: sudo docker compose up -d'
  exit 0
fi

# Rebuilds only changed layers, and recreates the container only if its image,
# compose.yaml or .env changed. A failed build leaves the old container running.
before=$(as_root docker compose ps -aq mapping)
as_root docker compose up -d --build --remove-orphans
after=$(as_root docker compose ps -aq mapping)

# config/ is mounted, so edits there only need a restart for ROS to reload them.
if [[ $before == "$after" ]] && { $force || grep -q '^config/' <<<"$changed"; }; then
  as_root docker compose restart mapping
fi

if grep -qx 'install.sh' <<<"$changed"; then
  echo 'install.sh changed; rerun "bash install.sh" to apply host-level changes.'
fi
if grep -q '^arduino_uno_rs/' <<<"$changed"; then
  echo 'Uno firmware changed; run "bash flash_uno.sh" to flash it (wheels off the ground).'
fi
echo 'Update applied.'
