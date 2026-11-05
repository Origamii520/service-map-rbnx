# Occupancy grid editing

Open **Maps → Edit grid** on the Mapping page (port 8091 by default). The
editor is also available at `/editor?map_id=<saved-map-id>`. It uses the
service's `MAPPING_MAPS_DIR`; no robot-specific paths, extra web server,
Scene database or Go2 package are required.

Paint obstacles, erase false obstacles to free space, mark unknown cells, draw
straight walls or filled rectangles, and undo/redo before saving. Brush size is
in metres. PGM and YAML exports refer to the same selected revision. Restore
creates an empty delta against the original while retaining previous revisions.
An outdated browser receives a conflict rather than overwriting newer work;
its unsaved drawing remains visible until the operator reloads or discards it.

## What each consumer uses

Automatic application is integrated with the **slam_toolbox** backend:

| Consumer | Input |
| --- | --- |
| Initial/global candidate search | Original saved occupancy + selected user delta |
| Optional AMCL initialization | The same revision, pinned for that initialization run |
| Initial candidate verification | The same edited occupancy |
| Continuous localization after handover | Original slam_toolbox pose graph and scans |
| Verification after handover / continuous localization trust | Original occupancy corresponding to the pose graph |
| Mapping Web UI and navigation occupancy capability | Effective edited occupancy during saved-map localization; live raw map during mapping |

The graph directory passed to slam_toolbox is unchanged. The editor never
rewrites `posegraph.posegraph`, `posegraph.data`, original occupancy files or
map-frame identity. No AMCL tracking mode, motion command, relocalization request,
watchdog policy or additional motion gate is introduced by saving an edit.
Saving during initial localization takes effect on the **next** initialization;
the current run finishes with the revision it started with.

The occupancy capability now resolves to `/robonix/map/navigation_grid` for
slam_toolbox. `/map` remains owned by SLAM / its initialization map server.
Consumers resolving `robonix/service/map/occupancy_grid` through Atlas pick up
the endpoint on restart/rebind. Deployments that explicitly override a map topic
must update that override to `/robonix/map/navigation_grid`, including Nav2's
static layer and any Scene/map displays. Do not point them at the raw `/map`
when edited navigation is intended. The relay runs with the existing Mapping
supervisor even if the web page is closed or the HTTP UI is disabled.

Other backends can edit and export saved grids, but the page explicitly labels
these **export only**; their running navigation and localization are not changed.
The storage/editor is robot independent. Automatic application currently follows
the slam_toolbox initialization/handover lifecycle, not every SLAM backend.

## Persistence

```text
<map-id>/
  occupancy.{pgm,png,yaml}       # original saved map, unchanged
  posegraph.{posegraph,data}    # original SLAM artifacts, unchanged
  .grid-edits/
    active                     # atomically replaced revision identifier
    lock                       # serializes writers across processes
    <revision>/
      delta.json               # authoritative sparse overrides against base
      occupancy.{pgm,png,yaml}  # derived, self-contained export/map_server input
```

`delta.json` schema 1 contains `base_sha256` and `cells: [[index, value], ...]`.
Indices address row-major image pixels **from the top left**, unlike the ROS
OccupancyGrid data array which starts at the bottom left. Values are grayscale
bytes. Every revision is relative to the immutable original, not chained to
another edit. Unpainted grayscale values are preserved exactly. The base hash
covers pixels, shape and YAML metadata, including origin, resolution, thresholds
and negate. A changed base cannot silently inherit edits from an older map.

Exports are prepared and synced before the revision and active pointer are
published. Reads use a complete revision. The initializer checks derived caches
against the delta before passing them to map_server. Back up the original map
and `.grid-edits` together. Do not copy only the active pointer.

To import an existing full edited PNG/PGM without modifying the original:

```bash
python3 scripts/import_grid_edit.py --maps-dir /path/to/maps \
  --map-id lab --image /path/to/edited/occupancy.pgm
```

The import prints the new revision and changed-cell count. It rejects dimension
changes and concurrent edits. Before changing deployments, verify the exported
image and map geometry match the intended version.

## Scope and verification

This editor preserves dimensions, resolution and origin (including yaw), and
supports opaque trinary grayscale maps with either negate value. The existing
coarse/fine global matcher assumes axis-aligned grids; this PR does not add
rotated-grid localization. Export/ROS publishing preserve the saved yaw.
Images are limited to 16 million cells and JSON uploads to 24 MB. No new Python
dependency is needed beyond the existing Pillow, NumPy and PyYAML runtime.

Removing a false obstacle can reopen a route. It cannot repair a warped pose
graph, odometry drift, wrong extrinsics or real obstacles detected by the live
sensors. Checkpoints, local SLAM rebuilding and Explore integration are separate
work. This change does not claim physical navigation acceptance.

Tests cover sparse reconstruction/restart, original artifact preservation,
revision conflicts, concurrent saves, failed activation, base changes, negate,
export geometry, HTTP routes, initial-map pinning and graph handover. The relay
can be exercised on an isolated localhost ROS domain using synthetic occupancy
messages; no chassis or navigation commands are required.
