#!/bin/bash

echo "=== Available Audio Recording Devices ==="

echo "ALSA Devices (use with device:=hw:CARD,DEVICE):"
arecord -l 2>/dev/null || echo "arecord not found. Install alsa-utils."

echo "PulseAudio Sources (use with device:=pulse):"
pactl list sources short 2>/dev/null || echo "pactl not found. Install pulseaudio-utils."