# service-map-rbnx provider configuration (config.spec, specVersion 1).
#
# Describes the `config:` block of a robot deployment entry:
#
# service:
#   - name: mapping
#     url: https://github.com/syswonder/service-map-rbnx
#     config: <the properties below>
#
# The block reaches the service through Driver(CMD_INIT); it is never read from
# a file or the environment. `status` is the deployment switch read by rbnx and
# is not part of this block.
#
# Ignored key: `platform`. Runtime target selection belongs to the package
# manifest (manifest:) and ROBONIX_MAPPING_FORCE / ROBONIX_MAPPING_PLATFORM.

specVersion: 1

description: >-
  SLAM mapping service that builds and serves a 2D occupancy grid, a point cloud
  and a map-frame pose from the sensor providers bound below. Saved maps can be
  reloaded for localization at startup or through load_map.

properties:
  sensor_providers:
    type: object
    x-group: Sensors
    description: >-
      Sensor role to the provider id (deployment entry name) that supplies it. A
      key enables that Mapping input and its value selects the provider; unbound
      roles are not used. The mapping must not be empty, every value must be a
      non-empty string, and keys other than the roles below are rejected at init.
      rtabmap needs lidar2d, lidar3d, scan_converter, or both rgb and depth;
      slam_toolbox needs lidar2d or scan_converter. Example: {lidar3d: roof_lidar,
      rgb: front_camera, depth: front_camera, odom: base_chassis}.
    properties:
      lidar2d:
        type: string
        x-provider: robonix/primitive/lidar/lidar
        description: >-
          2D LaserScan provider. Mutually exclusive with scan_converter.
      lidar3d:
        type: string
        x-provider: robonix/primitive/lidar/lidar3d
        description: 3D lidar PointCloud2 provider.
      scan_converter:
        type: string
        x-provider: robonix/service/lidar/scan_converter/scan
        description: >-
          LaserScan output of service-pcld2lscan-rbnx, used in place of a native 2D
          lidar. Mutually exclusive with lidar2d.
      rgb:
        type: string
        x-provider: robonix/primitive/camera/rgb
        description: >-
          RGB camera provider. RGB-D input is used only when depth is also bound.
      depth:
        type: string
        x-provider: robonix/primitive/camera/depth
        description: >-
          Depth camera provider. RGB-D input is used only when rgb is also bound.
      imu:
        type: string
        x-provider: robonix/primitive/imu/imu
        description: >-
          IMU provider. RTAB-Map uses it, after Madgwick filtering, only when no odom
          is bound and its internal ICP odometry runs on a lidar.
      odom:
        type: string
        x-provider: robonix/primitive/chassis/odom
        description: >-
          External odometry provider. When bound, RTAB-Map uses it instead of its
          internal odometry and the service does not declare
          robonix/service/map/odom. Must not be bound when navigation_odom_bridge is
          true.

  algo:
    type: string
    x-group: Engine
    default: rtabmap
    enum: [rtabmap, slam_toolbox, dlio, fastlio2]
    description: >-
      Mapping engine. rtabmap is sensor-agnostic (2D lidar, 3D lidar, RGB-D) and
      stores its map in a database. slam_toolbox is 2D-lidar-only and CPU-only and
      serializes its pose graph to two files instead of a database. dlio and
      fastlio2 run fixed LiDAR-inertial launch files and do not support
      localization mode; fastlio2 has known drift and is retained only for
      diagnostics. The value is case-sensitive.

  base_frame:
    type: string
    x-group: Frames and time
    default: base_link
    description: >-
      Robot body frame used by the SLAM engine, the localizer and sensor
      transforms. It must match the complete robot URDF/TF tree and the frame used
      by Navigation. An empty string means base_link.

  odom_frame:
    type: string
    x-group: Frames and time
    default: odom
    description: >-
      Local continuous-motion frame used for odometry and map-to-odom estimation.
      The bound odom provider must publish poses in this frame. With
      navigation_odom_bridge it is the private RTAB-Map odometry frame and must
      differ from navigation_odom_frame (for example odom_icp). An empty string
      means odom.

  use_sim_time:
    type: boolean
    x-group: Frames and time
    default: false
    description: >-
      Use the ROS /clock source instead of wall time. Enable this for a simulator
      only when every sensor, TF publisher, and consumer uses the same simulated
      clock.

  map_frame:
    type: string
    x-group: Frames and time
    default: map
    description: >-
      Only for the slam_toolbox backend: map frame given to the slam_toolbox
      localization node that load_map starts, and frame of the pose it is seeded
      with. The slam_toolbox mapping node, RTAB-Map, the particle-filter
      localizer, the pose adapter and the web UI always use map, so any other value
      leaves the frames inconsistent. Leave it unset.

  map_mode:
    type: string
    x-group: Startup map
    default: mapping
    enum: [mapping, localization]
    description: >-
      Startup mode; switch_mode and load_map can change it while the service runs.
      mapping always starts a fresh mutable session (a new runtime database for
      rtabmap). localization loads the saved map selected by map_id: rtabmap copies
      its rtabmap.db into a runtime database and localizes against that copy, so
      the saved artifact is not modified; slam_toolbox starts idle and runs the
      load_map handover in the background. localization requires algo rtabmap or
      slam_toolbox. The value is case-insensitive.

  map_id:
    type: string
    x-group: Startup map
    description: >-
      Saved spatial-map identifier to load when map_mode is localization; required
      in that mode and ignored in mapping mode, where new maps are named by the
      save_map operation. The saved map lives in MAPPING_MAPS_DIR/<map_id>, must
      have been built by the same algo, and must contain that engine's artifacts
      (rtabmap.db, or posegraph.posegraph and posegraph.data). Characters other
      than letters, digits, '.', '_' and '-' are replaced with '_'.

  localizer:
    type: string
    x-group: Localization
    default: none
    enum: [none, amcl, beluga]
    description: >-
      Only for the slam_toolbox backend; RTAB-Map relocalizes against its own
      database and ignores this key. With none, load_map has slam_toolbox
      deserialize its pose graph and localize from the pose given to load_map,
      which must be close enough for scan matching to converge. amcl (nav2_amcl)
      and beluga (beluga_amcl, interface-compatible) let load_map relocalize with
      no prior pose: the current scan is first matched against the saved occupancy grid
      with the robot standing still, and if that is inconclusive the particle
      filter runs over the grid while the robot is driven. The filter never
      publishes map to odom; the recovered pose is handed to the slam_toolbox
      localization node. It consumes the scan from lidar2d or scan_converter
      (MAPPING_SCAN_TOPIC or /scan when neither is resolved). CPU only, a few tens
      of MB. The value is case-insensitive.

  localizer_particles:
    type: object
    x-group: Localization
    description: >-
      Particle counts for the amcl or beluga localizer. More particles converge
      from worse guesses and cost proportionally more. Other keys are ignored.
    properties:
      min:
        type: integer
        default: 500
        description: Minimum particle count. 0 is treated as absent.
      max:
        type: integer
        default: 2000
        description: Maximum particle count. 0 is treated as absent.

  params_file:
    type: string
    x-group: RTAB-Map
    description: >-
      Only for the rtabmap backend. Path to a deploy-owned YAML mapping of RTAB-Map
      parameters (string keys, values are strings, numbers or booleans), relative
      to the directory containing robonix_manifest.yaml; absolute paths and ~ are
      accepted. Init fails when the file is missing or is not such a mapping. Copy
      the upstream config/rtabmap_params.template.yaml into the deploy repository
      as a starting point; the upstream template is never loaded at runtime.
      Values from this file override the launch defaults and are in turn
      overridden by rtabmap_params.

  rtabmap_params:
    type: object
    x-group: RTAB-Map
    description: >-
      Only for the rtabmap backend. Final RTAB-Map parameter overrides, applied
      after params_file. Keys are non-empty RTAB-Map or node parameter names such
      as Grid/FootprintLength; values are strings, numbers or booleans (lists,
      mappings and null are rejected). Icp/, Odom/ and Reg/ keys, and native
      parameters the launch already sets on the ICP odometry node, are also
      passed to that node. publish_null_when_lost must be a YAML boolean.

  occupancy_sources:
    type: array
    x-group: RTAB-Map
    items:
      type: string
      enum: [lidar, depth]
    description: >-
      Only for the rtabmap backend. Inputs used to build the 2D occupancy grid
      (sets Grid/Sensor and Grid/FromDepth). Must be non-empty, and every listed
      source must be resolved: lidar needs lidar2d, lidar3d or scan_converter,
      depth needs rgb and depth. Cannot be combined with Grid/Sensor or
      Grid/FromDepth in params_file or rtabmap_params. When absent, the grid uses
      every resolved lidar and RGB-D input.

  rtabmap_inputs:
    type: array
    x-group: RTAB-Map
    items:
      type: string
      enum: [lidar, rgbd, imu, odom]
    description: >-
      Only for the rtabmap backend. Subset of the resolved providers passed into
      RTAB-Map; must be non-empty, and init fails when a listed input was not
      resolved. When absent, every resolved provider is used. New deployments
      normally omit this and bind only the providers they use.

  deskew_lidar:
    type: boolean
    x-group: RTAB-Map
    default: false
    description: >-
      Only for the rtabmap backend. Deskew the lidar3d PointCloud2; the cloud must
      carry per-point timestamps, and the launch fails without a lidar3d input.
      With a bound odom provider a deskewing node uses the odom_frame TF;
      otherwise the internal ICP odometry deskews.

  navigation_odom_bridge:
    type: boolean
    x-group: RTAB-Map
    default: false
    description: >-
      Only for the rtabmap backend. Split-odometry mode for robots whose accurate
      mapping odometry is too latent for Navigation. Internal RTAB-Map odometry
      becomes a message-only private trajectory in odom_frame (no TF), and a
      bridge publishes map to navigation_odom_frame by combining RTAB-Map
      localization with the chassis pose from navigation_odom_topic. RViz
      /initialpose is forwarded to RTAB-Map by the bridge. Requires no
      sensor_providers.odom binding and an odom_frame different from
      navigation_odom_frame. false keeps the default TF behaviour.

  navigation_odom_topic:
    type: string
    x-group: RTAB-Map
    default: /odom
    description: >-
      Chassis Odometry topic read by the bridge; used only when
      navigation_odom_bridge is true. It is not an RTAB-Map sensor input and does
      not require a sensor_providers.odom binding.

  navigation_odom_frame:
    type: string
    x-group: RTAB-Map
    default: odom
    description: >-
      Frame owned by the chassis navigation odometry; used only when
      navigation_odom_bridge is true, and then must differ from odom_frame.

  slam_toolbox_params:
    type: object
    x-group: slam_toolbox
    description: >-
      Only for the slam_toolbox backend; the counterpart of rtabmap_params.
      Scan-matching overrides applied to both the mapping and the localization
      node. Other keys are rejected at init, and every value must be a number
      greater than 0. Unset keys keep the defaults, which suit a slow indoor
      platform in a room-scale map.
    properties:
      min_travel_m:
        type: number
        default: 0.1
        description: >-
          Distance in metres the robot moves before a new pose-graph node is added.
          Must be greater than 0.
      min_heading_rad:
        type: number
        default: 0.1
        description: >-
          Rotation in radians before a new pose-graph node is added. Must be
          greater than 0.
      scan_buffer:
        type: integer
        minimum: 1
        default: 30
        description: >-
          Number of recent scans matched against each other. scan_buffer times
          min_travel_m should stay near the room scale. A fractional value is
          truncated.
      loop_search_m:
        type: number
        default: 2.5
        description: >-
          Radius in metres searched for a loop closure. Must be greater than 0.

  editor_max_pixels:
    type: integer
    x-group: Web UI
    default: 16000000
    minimum: 1
    description: >-
      Maximum decoded image width times height, in pixels (occupancy cells),
      for the grid editor and saved edit loading. This is not a byte limit.
      Increase editor_max_body_bytes as needed for the base64 PNG plus JSON;
      PNG compression varies, so pixel count alone does not determine upload size.

  editor_max_body_bytes:
    type: integer
    x-group: Web UI
    default: 24000000
    minimum: 1
    description: >-
      Maximum complete editor save/restore JSON request body in bytes, including
      base64 PNG and metadata, excluding HTTP headers. Base64 uses 4 times
      ceil(PNG bytes / 3), plus JSON overhead. Independent of editor_max_pixels;
      both limits must accommodate a save. Defaults use decimal bytes, not MiB.

  webui_port:
    type: [integer, string]
    x-group: Web UI
    default: 8091
    description: >-
      TCP port of the Mapping operator web UI. 0, "0" or an empty string disables
      it and also clears any inherited MAPPING_WEBUI_PORT. A value that cannot be
      bound leaves the page off with a warning in the log.

  webui_host:
    type: string
    x-group: Web UI
    default: 127.0.0.1
    description: >-
      Bind address for the unauthenticated Mapping web UI. Keep the loopback
      default unless an authenticated deployment overlay protects access. When the
      key is absent, MAPPING_WEBUI_HOST from the service environment is used if
      set; an empty string means 127.0.0.1.

  webui_scan_topic:
    type: string
    x-group: Web UI
    description: >-
      LaserScan topic drawn as the live overlay on the web UI map, for a
      deployment whose 2D scan is not an Atlas capability. When absent or empty,
      the scan resolved from lidar2d or scan_converter is used, and failing that
      a LaserScan topic found on the ROS graph. MAPPING_WEBUI_SCAN_TOPIC in the
      service environment takes precedence over this key.

  reset_map:
    type: boolean
    x-group: Deprecated
    default: false
    description: >-
      Deprecated with no replacement: every mapping session already starts with a
      fresh runtime database, so it has no effect in mapping mode. true is
      rejected in localization mode. New deployments omit it.

  sensors:
    type: object
    x-group: Deprecated
    description: >-
      Deprecated; replaced by sensor_providers. Legacy role table of booleans
      (1, true, yes and on count as true), read only when sensor_providers is
      absent. At least one role must be true, and using the table logs a migration
      warning. With no provider id, init fails when more than one Atlas provider
      exposes a role's contract.
    properties:
      lidar2d: {type: boolean, description: Enable the 2D lidar role.}
      lidar3d: {type: boolean, description: Enable the 3D lidar role.}
      scan_converter: {type: boolean, description: Enable the scan_converter role.}
      rgb: {type: boolean, description: Enable the RGB camera role.}
      depth: {type: boolean, description: Enable the depth camera role.}
      imu: {type: boolean, description: Enable the IMU role.}
      odom: {type: boolean, description: Enable the external odometry role.}

  rtabmap_profile:
    type: string
    x-group: Deprecated
    enum: [ranger_mini_v3, webots_tiago]
    description: >-
      Deprecated; replaced by params_file or rtabmap_params. Only for the rtabmap
      backend. Applies a frozen legacy parameter set with a migration warning;
      params_file and rtabmap_params override its values. Unknown names are
      rejected at init.

required: [sensor_providers]
