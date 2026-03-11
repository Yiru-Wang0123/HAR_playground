#!/bin/bash
# install_service.sh — Run this ONCE on the Pi to enable auto-start
#
# Usage:
#   scp hr_distance.service install_service.sh IMST@10.0.0.39:~/wyr/
#   ssh IMST@10.0.0.39 'bash ~/wyr/install_service.sh'

set -e

SERVICE_FILE="$HOME/wyr/hr_distance.service"
SERVICE_NAME="hr_distance.service"

# Create trials directory
mkdir -p "$HOME/wyr/trials"

# Copy service file to systemd
sudo cp "$SERVICE_FILE" /etc/systemd/system/"$SERVICE_NAME"

# Reload systemd, enable on boot, start now
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl start "$SERVICE_NAME"

echo ""
echo "Done! hr_distance.py will now start on every boot."
echo ""
echo "Useful commands:"
echo "  sudo systemctl status hr_distance    # check if running"
echo "  sudo journalctl -u hr_distance -f    # live logs"
echo "  sudo systemctl stop hr_distance      # stop it"
echo "  sudo systemctl restart hr_distance   # restart it"
echo "  ls ~/wyr/trials/                     # see saved CSVs"
