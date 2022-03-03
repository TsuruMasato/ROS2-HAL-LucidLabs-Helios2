# LucidLabs Helios2/Helios2+ ToF (Time-of-Flight) camera ROS2 HAL

## Prerequisites
Please download SDK from LucidLabs [support page](https://thinklucid.com/downloads-hub) and unpack `ArenaSDK_Linux_x64` folder somewhere in your system (good places are either `/usr/local`, `/opt`, `~/bin`).

Open `.bashrc` and add the following lines to the end:
```
export ARENA_GCCVER=54_v3_3_LUCID
export ARENA_PATH=/path/to/ArenaSDK_Linux_x64
```
The `ARENA_GCCVER` is the file suffix of the files present in `ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64`.

Finally, open a new terminal or source `.bashrc` before running `colcon`.
