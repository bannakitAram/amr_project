# Put your saved map here

Copy the two files produced by `map_saver_cli` (see the SLAM step in the
top-level README) into this folder:

```bash
cp ~/amr_map.yaml ~/amr_map.pgm amr_ws/src/amr_navigation/maps/
```

Then rebuild so they get installed:

```bash
cd amr_ws
colcon build --symlink-install --packages-select amr_navigation
source install/setup.bash
```

By default `navigation.launch.py` looks for `maps/amr_map.yaml` in this
package. Rename your files to `amr_map.yaml` / `amr_map.pgm` to match, or
pass a different path with:

```bash
ros2 launch amr_navigation navigation.launch.py map:=/full/path/to/your_map.yaml
```
