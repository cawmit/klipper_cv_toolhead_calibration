#!/bin/bash

KLIPPER_PATH="${HOME}/klipper"

link_extension()
{
    echo "Linking extension to Klipper..."
    ln -sf "${SRCDIR}/cv_toolhead_calibration.py" "${KLIPPER_PATH}/klippy/extras/cv_toolhead_calibration.py"
}

verify_opencv_installed()
{
    sudo apt-get install -y python-opencv
}

restart_klipper()
{
    echo "Restarting Klipper..."
    sudo systemctl restart klipper
}

verify_ready()
{
    if [ "$EUID" -eq 0 ]; then
        echo "This script must not run as root"
        exit -1
    fi
}

# Force script to exit if an error occurs
set -e

# Find SRCDIR from the pathname of this script
SRCDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/ && pwd )"

# Parse command line arguments
while getopts "k:" arg; do
    case $arg in
        k) KLIPPER_PATH=$OPTARG;;
    esac
done

verify_ready
verify_opencv_installed
link_extension
restart_klipper