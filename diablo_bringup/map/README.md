# Saved maps

The web mapping interface writes each saved map here as a matching pair:

- `<name>.pgm` — occupancy image
- `<name>.yaml` — map metadata used by ROS 2 map servers

Generated map files are intentionally kept outside the web package so they can
be reused by future navigation bringup without changing the mapping UI.
