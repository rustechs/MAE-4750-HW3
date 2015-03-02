#!/bin/bash

echo "Make sure you're connected to the ethernet in the lab"
echo "Attempting to connect..."
sudo dhclient eth0
