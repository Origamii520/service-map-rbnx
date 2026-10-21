# SPDX-License-Identifier: MulanPSL-2.0
"""Real process groups: launch leader exit must not leave a filter worker."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from mapping_rbnx import localizers


class LocalizerProcessCleanupTests(unittest.TestCase):
    def test_exited_launch_leader_still_has_its_stubborn_child_stopped(self):
        # No ROS: reproduce a child that outlives its ros2-launch group leader.
        with tempfile.TemporaryDirectory() as tmp:
            marker = Path(tmp)/'child.pid'
            code = '''import os,signal,sys,time
pid=os.fork()
if pid:
    while not os.path.exists(sys.argv[1]): time.sleep(.01)
    sys.exit(0)
signal.signal(signal.SIGINT,signal.SIG_IGN)
signal.signal(signal.SIGTERM,signal.SIG_IGN)
open(sys.argv[1],'w').write(str(os.getpid()))
while True: time.sleep(.1)
'''
            proc = subprocess.Popen([sys.executable,'-c',code,str(marker)], start_new_session=True)
            child = None
            try:
                proc.wait(timeout=3)
                child = int(marker.read_text())
                self.assertEqual(os.getpgid(child),proc.pid)
                localizers._PROC = proc
                localizers._ACTIVE_MAP = 'fixture'
                localizers.stop()
                deadline = time.monotonic()+2
                while time.monotonic()<deadline:
                    path=Path(f'/proc/{child}/stat')
                    if not path.exists() or path.read_text().rsplit(')',1)[1].split()[0]=='Z':
                        break
                    time.sleep(.02)
                else:
                    self.fail('localizer child survived after launch leader exited')
                self.assertIsNone(localizers._PROC)
                self.assertEqual(localizers.active_map(),'')
            finally:
                try: os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError: pass
                proc.wait(timeout=3)
                localizers._PROC=None


if __name__=='__main__':
    unittest.main()
