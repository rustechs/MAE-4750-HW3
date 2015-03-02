#!/bin/bash

echo "\nMake sure you're on the lab ethernet"
echo "Attempting to connect to baxter...\n"

sudo dhclient eth0
