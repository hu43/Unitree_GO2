#!/bin/bash
# Install the Go2 network helper (host side, run as root).
#   sudo ./install-net-helper.sh
set -e

SRC="$(cd "$(dirname "$0")" && pwd)"

install -m 755 "$SRC/go2-net-helper.py" /usr/local/bin/go2-net-helper.py
install -m 644 "$SRC/go2-net-helper.service" /etc/systemd/system/go2-net-helper.service
install -m 644 "$SRC/go2-net-helper.timer" /etc/systemd/system/go2-net-helper.timer

# spool dirs shared with the web app (bind-mounted into the container)
mkdir -p /home/unitree/hu/go2/net-ctl/queue /home/unitree/hu/go2/net-ctl/results
chmod 755 /home/unitree/hu/go2/net-ctl

systemctl daemon-reload
systemctl enable --now go2-net-helper.timer
systemctl start go2-net-helper.service || true

echo "installed. status:"
systemctl is-active go2-net-helper.timer
ls -la /home/unitree/hu/go2/net-ctl/
