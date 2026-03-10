# rear_mic_recorder

ROS package to record WAV audio from the rear built-in microphone using ALSA `arecord`.

## Usage

```bash
roslaunch rear_mic_recorder record_rear_mic.launch
```

## Launch parameters

- `device` default: `default`
- `sample_rate` default: `44100`
- `channels` default: `2`
- `sample_format` default: `S16_LE`
- `duration` default: `0` (0 means record until node shutdown)
- `output_dir` default: `data`
- `filename_prefix` default: `rear_mic`
- `filename` default: `""` (auto timestamped if empty)

## Notes

- Requires `arecord` (`alsa-utils`) to be installed.
- To inspect devices: `arecord -l`
- Relative `output_dir` values are resolved from the repository root, so the default writes into `auto_liver_ultrasound/data`.
