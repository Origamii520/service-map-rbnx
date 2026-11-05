# SPDX-License-Identifier: MulanPSL-2.0
"""Separate occupancy output; never publish edited cells into SLAM's /map."""
import logging
import math
import os

from .grid_edits import Store, map_directory

TOPIC = '/robonix/map/navigation_grid'
log = logging.getLogger(__name__)


class Selection:
    """Cache one saved revision; mapping sessions always use the live source."""
    def __init__(self, root):
        self.root, self.key, self.saved = root, None, None

    def get(self, state):
        if state.get('mode') != 'localization' or not state.get('map_id'):
            self.key, self.saved = None, None
            return None
        path = map_directory(self.root, state['map_id'])
        active = path / '.grid-edits' / 'active'
        def stamp(p):
            try:
                s = p.stat()
                return s.st_mtime_ns, s.st_size, s.st_ino
            except FileNotFoundError:
                return None
        key = (str(path), state.get('generation'), stamp(active), stamp(path / 'occupancy.yaml'), stamp(path / 'occupancy.pgm'))
        if key != self.key:
            store = Store(path)
            revision = store.revision()
            self.saved = (store, store.occupancy(revision), revision)
            self.key = key
        return self.saved


class Relay:
    def __init__(self, node, source, root, state, on_grid, qos):
        from nav_msgs.msg import OccupancyGrid
        self.node, self.state, self.on_grid = node, state, on_grid
        self.cls, self.selection = OccupancyGrid, Selection(root)
        self.source, self.epoch, self.error = None, None, None
        self.status = {}
        self.pub = node.create_publisher(OccupancyGrid, TOPIC, qos)
        self.sub = node.create_subscription(OccupancyGrid, source, self.receive, qos)
        self.timer = node.create_timer(.5, self.publish)

    def receive(self, msg):
        state = self.state()
        self.epoch = (state.get('map_id'), state.get('generation'), state.get('mode'))
        self.source = msg
        self.publish()

    def publish(self):
        try:
            state = self.state()
            selected = self.selection.get(state)
            if selected is None:
                epoch = (state.get('map_id'), state.get('generation'), state.get('mode'))
                if self.source is None or epoch != self.epoch:
                    return
                msg = self.source
            else:
                store, cells, revision = selected
                msg = self.cls()
                msg.header.frame_id = os.environ.get('MAPPING_MAP_FRAME', 'map')
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.info.width, msg.info.height = store.width, store.height
                msg.info.resolution = store.resolution
                msg.info.origin.position.x, msg.info.origin.position.y = store.origin[:2]
                msg.info.origin.orientation.z = math.sin(store.origin[2] / 2)
                msg.info.origin.orientation.w = math.cos(store.origin[2] / 2)
                msg.data = cells
            self.status = dict(map_id=state.get('map_id', ''),
                               revision=selected[2] if selected else 'live', error='')
            self.pub.publish(msg)
            self.on_grid(msg)
            self.error = None
        except Exception as exc:
            # Do not substitute an older edited map for an unreadable new one.
            self.status = dict(self.status, error=str(exc))
            if str(exc) != self.error:
                log.error('cannot publish selected occupancy grid: %s', exc)
                self.error = str(exc)
