# maps/

This directory holds saved SLAM maps. The placeholder `empty.yaml` +
`empty.pgm` is what `mode:=navigation` falls back to when no real
map is supplied via the `world:=` argument.

To produce a real map of an indoor environment, follow the procedure
in `../../docs/MAPPING.md`. The deliverable is two files:

```
<your_map>.yaml
<your_map>.pgm
```

Commit them here and pass `world:=...` to
`ros2 launch rover_bringup rover.launch.py mode:=navigation`.
