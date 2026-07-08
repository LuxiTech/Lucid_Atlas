# Skill: OctoMap to OccupancyGrid Converter

## Goal

Convert a 3D OctoMap into a 2D ROS navigation map.

---

## Input

```text
map.bt
```

---

## Output

```text
map.pgm
map.yaml
```

---

## Workflow

```text
OctoMap
 ↓
Projection
 ↓
OccupancyGrid
 ↓
map.pgm
map.yaml
```

---

## Command Line

```bash
octomap2grid \
    --input map.bt \
    --resolution 0.05 \
    --output ./maps
```

---

## Parameters

```yaml
input:
  octomap file

output:
  output directory

resolution:
  occupancy grid resolution

occupied_threshold:
  occupied threshold

free_threshold:
  free threshold
```

---

## Projection Rules

Each voxel is classified as:

```text
Occupied
Free
Unknown
```

The 3D map is projected onto the XY plane.

---

## Generated Files

```text
maps/
├── map.pgm
├── map.yaml
```

---

## Example YAML

```yaml
image: map.pgm
resolution: 0.05
origin: [-20.0,-20.0,0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

---

## Supported Navigation Systems

* ROS1 move_base
* ROS2 Nav2
* AMCL
* Global Planner
* Local Planner

```
```
