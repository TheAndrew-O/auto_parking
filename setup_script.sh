#!/bin/bash

REPO_URL="https://github.com/TheAndrew-O/auto_parking.git"
TEMP_DIR="/home/ubuntu/Desktop/auto_parking"
TARGET_DIR="/home/ubuntu"

# Install Python package xacro
echo "Installing Python package 'xacro'..."
pip install xacro || { echo "Error: Failed to install xacro."; exit 1; }

# Clone auto_parking repository
echo "Cloning auto_parking repository..."
#cd home/ubuntu/
git clone "$REPO_URL" "$TEMP_DIR" || { echo "Error: Failed to clone auto_parking repository."; exit 1; }

echo "Moving to target directory"
mv "$TEMP_DIR" "$TARGET_DIR" || { echo "Error: Failed to move auto_parking directory."; exit 1; }
#rm -rf "$TEMP_DIR"
#Sleep
#echo "Sleeping for 5 seconds..."
#sleep 5

# Copy parking_lot directory to Gazebo models directory
echo "Copying parking_lot directory to Gazebo models directory..."
sudo cp -r /home/ubuntu/auto_parking/car_ws/src/auto_park2/worlds/parking_lot /usr/share/gazebo-11/models/ || { echo "Error: Failed to copy parking_lot directory."; exit 1; }

echo "Script execution completed successfully."

