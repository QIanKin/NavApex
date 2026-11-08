"""Run: python3 -m unittest discover -s tools/log_viewer -p 'test_*.py'."""
import base64
import gzip
import json
import tempfile
import threading
import unittest
import urllib.request
from pathlib import Path

from main import make_server
from replay_log import ReplayLog, sample_trajectory

SESSION = "1794132000000000000-123"
META = {"v": 1, "robot": {"shape": "circle", "radius": 0.125}, "frame": "map"}


def record(seq, **fields):
    value = dict(v=1, stamp=seq * 0.1, **fields)
    payload = json.dumps(value).replace("\\", "\\\\").replace("\n", "\\n").replace("\r", "\\r")
    return f'18:00:00 100(123)I/replay: epoch_ns={1794132000000000000+seq*100000000} seq={seq} channel=replay tid=1 source="replay" line=0 function="record" | {payload}\n'


def header():
    return f'# session_id={SESSION} software_version="test"\n# replay_metadata={json.dumps(META)}\n'


def grid(seq=1, map_id=1, data=None):
    data = bytes(range(256)) if data is None else data
    return record(seq, event="costmap", map_id=map_id, layer="local", width=16, height=16,
                  resolution=0.02, origin=[-1.6, -1.6], layout="row_major_y_x", encoding="gzip+base64",
                  data=base64.b64encode(gzip.compress(data)).decode())


def plan(seq=2, map_id=1):
    return record(seq, event="plan", plan_id=seq, map_id=map_id, state_id=101,
                  state={"pose": [0.3, 0, 0]}, success=True, accepted=True,
                  status="planning_success", elapsed_ms=1.0, command=[0.2, 0],
                  trajectory={"pieces": [{"duration": 1, "coefficients": [[0, 0, 0, 0, 2, 3], [0]*6, [0]*6]}]})


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)

    def file(self, name, content):
        path = Path(self.temp.name) / name
        path.write_text(content)
        return path

    def load(self, *paths):
        model = ReplayLog()
        for path in paths:
            model.add_file(path)
        model.finalize()
        return model

    def test_lossless_grid_all_cost_values(self):
        model = self.load(self.file("a.log", header() + grid()))
        self.assertEqual(model.map_bytes(SESSION, 1), bytes(range(256)))
        self.assertEqual(model.summary()["groups"][0]["metadata"]["robot"]["radius"], .125)

    def test_exact_reference_and_backward_seek(self):
        model = self.load(self.file("a.log", header()+grid()+grid(2,2)+plan(3,1)+plan(4,2)))
        key = model.summary()["groups"][0]["key"]
        first = model.frame(key, 0)
        model.frame(key, 1)
        self.assertEqual(first, model.frame(key, 0))
        self.assertEqual(first["maps"][0]["map_id"], 1)  # Never replace with newest map 2.
        self.assertEqual(first["state"]["pose"][0], .3)
        self.assertEqual(first["plan"]["trajectory"][0][0], 3)
        self.assertEqual(first["plan"]["trajectory"][-1][0], 5)

    def test_rotated_files_unordered_and_duplicate(self):
        a = self.file("a.log.1", header()+grid())
        b = self.file("a.log", header()+plan())
        model = self.load(b, a, a)
        self.assertEqual(len(model.records), 2)
        self.assertEqual(model.map_bytes(SESSION, 1), bytes(range(256)))

    def test_missing_map_never_substituted(self):
        model = self.load(self.file("a.log", header()+grid()+plan(map_id=999)))
        frame = model.frame(model.summary()["groups"][0]["key"], 0)
        self.assertFalse(frame["maps"])
        self.assertTrue(frame["warnings"])

    def test_corrupt_compressed_map(self):
        model = self.load(self.file("a.log", header()+grid(data=b"short")))
        with self.assertRaises(ValueError):
            model.map_bytes(SESSION, 1)

    def test_truncated_and_malformed_lines(self):
        model = self.load(self.file("a.log", header()+grid()+record(2,event="pose",state={"pose":"bad"})+plan().rstrip()))
        self.assertEqual(len(model.records), 1)
        self.assertEqual(sum(model.warnings.values()), 2)

    def test_incomplete_plan_does_not_break_viewer(self):
        model = self.load(self.file("a.log", header()+grid()+record(2,event="plan",plan_id=1)))
        self.assertEqual(len(model.records), 1)
        self.assertTrue(model.warnings)

    def test_waiting_for_first_input_does_not_invent_pose(self):
        event = record(2, event="plan", plan_id=1, map_id=0, state_id=0, state=None,
                       success=False, accepted=True, elapsed_ms=0, command=[0,0], status="waiting_for_fresh_inputs")
        model = self.load(self.file("a.log", header()+event))
        frame = model.frame(model.summary()["groups"][0]["key"], 0)
        self.assertIsNone(frame["state"])
        self.assertIn("等待输入", frame["warnings"][0])

    def test_simulation_reset_is_separate_segment(self):
        model = self.load(self.file("a.log", header()+grid()+record(2,event="pose",clock_epoch=1,state={"pose":[0,0,0]})))
        self.assertEqual(len(model.summary()["groups"]), 2)

    def test_text_escaping_and_legacy(self):
        model = self.load(self.file("old.log", header()+'18:00:00 100(123)D/test: path=C:\\\\log newline=\\n quote="ok"\n'))
        self.assertIn('C:\\log', model.diagnostics[0]["text"])
        self.assertIn('\n', model.diagnostics[0]["text"])
        self.assertTrue(model.summary()["warnings"])

    def test_source_precision_and_local_polynomial_time(self):
        pieces = {"pieces": [{"duration": 2, "coefficients": [[0,0,0,0,1,2],[0]*6,[0]*6]},
                               {"duration": 1, "coefficients": [[0,0,0,0,2,4],[0]*6,[0]*6]}]}
        points = sample_trajectory(pieces)
        self.assertEqual(points[0][0],2)
        self.assertEqual(points[-1][0],6)
        with self.assertRaises(ValueError):
            sample_trajectory({"pieces":[{"duration":-1,"coefficients":[[0]*6]*3}]})

    def test_http_upload_frame_and_map(self):
        server = make_server(0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            request = urllib.request.Request(base+'/api/import?name=test.log&reset=1',
                (header()+grid()+plan()).encode(), method='POST')
            with urllib.request.urlopen(request) as response:
                self.assertEqual(response.status, 200)
            with urllib.request.urlopen(base+'/api/summary') as response:
                summary = json.load(response)
            self.assertEqual(summary['records'], 2)
            with urllib.request.urlopen(base+f'/api/map?session={SESSION}&id=1') as response:
                self.assertEqual(response.read(), bytes(range(256)))
        finally:
            server.shutdown(); server.server_close(); server.temporary.cleanup();thread.join()


if __name__ == "__main__":
    unittest.main()
