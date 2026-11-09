"""NavApex replay v1 reader. Only the Python standard library is required."""
from __future__ import annotations

import base64
import bisect
import collections
import datetime as dt
import gzip
import io
import json
import math
import re
from pathlib import Path

MAX_LINE = 8 * 1024 * 1024
MAX_CELLS = 16 * 1024 * 1024
PREFIX = re.compile(r"^(\d\d:\d\d:\d\d) (\d{3})\(\d+\)([DIWE])/([^:]+): (.*)$")
REPLAY = re.compile(r"epoch_ns=(\d+) seq=(\d+) channel=replay .*? \| (.*)$")


def unescape(text):
    # Undo the outer logger escaping once; JSON escapes are decoded by json.loads.
    return re.sub(r"\\([\\nr])", lambda m: {"\\": "\\", "n": "\n", "r": "\r"}[m[1]], text)


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def vector(value, size):
    return isinstance(value, list) and len(value) == size and all(finite(x) for x in value)


def reject_constant(value):
    raise ValueError("non-finite JSON constant: " + value)


def sample_trajectory(value):
    """Evaluate descending coefficients in each piece's local seconds."""
    if not value:
        return []
    pieces = value.get("pieces", [])
    if not isinstance(pieces, list) or len(pieces) > 1000:
        raise ValueError("invalid trajectory pieces")
    result = []
    total = sum(p.get("duration", 0) for p in pieces)
    step = max(0.05, total / 4000)
    for piece in pieces:
        duration, coefficients = piece["duration"], piece["coefficients"]
        if not finite(duration) or duration <= 0 or len(coefficients) != 3:
            raise ValueError("invalid trajectory duration/axes")
        if any(len(axis) != 6 or not all(finite(c) for c in axis) for axis in coefficients):
            raise ValueError("invalid trajectory coefficients")
        count = max(1, math.ceil(duration / step))
        for i in range(count + 1):
            t = duration * i / count
            point = []
            for axis in coefficients:
                v = 0.0
                for coefficient in axis:
                    v = v * t + coefficient
                point.append(v)
            if not all(finite(v) for v in point):
                raise ValueError("non-finite trajectory sample")
            result.append(point)
    return result


class ReplayLog:
    def __init__(self):
        self.records = []
        self.diagnostics = []
        self.metadata = {}
        self.warnings = collections.Counter()
        self.seen = set()
        self.groups = {}
        self.maps = {}
        self.files = []
        self._decoded = collections.OrderedDict()

    def add_file(self, path, name=None):
        path = Path(path)
        name = name or path.name
        self.files.append(name)
        session = name
        wall_day = None
        with path.open("rb") as stream:
            while True:
                offset = stream.tell()
                raw = stream.readline(MAX_LINE + 1)
                if not raw:
                    break
                if len(raw) > MAX_LINE:
                    self.warnings["超长日志行已跳过"] += 1
                    while raw and not raw.endswith(b"\n"):
                        raw = stream.readline(MAX_LINE + 1)
                    continue
                if not raw.endswith(b"\n"):
                    self.warnings["末尾不完整记录已跳过"] += 1
                    continue
                text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                if text.startswith("# session_id="):
                    session = text.split("=", 1)[1].split()[0]
                    try:
                        wall_day = dt.datetime.fromtimestamp(int(session.split("-")[0]) / 1e9).date()
                    except (ValueError, OverflowError, OSError):
                        pass
                    continue
                if text.startswith("# replay_metadata="):
                    try:
                        metadata = json.loads(unescape(text.split("=", 1)[1]), parse_constant=reject_constant)
                        if not isinstance(metadata, dict):
                            raise ValueError("metadata is not an object")
                        if "robot" in metadata:
                            robot = metadata["robot"]
                            if (not isinstance(robot, dict) or not isinstance(robot.get("shape"), str) or
                                    not finite(robot.get("radius")) or robot["radius"] <= 0):
                                raise ValueError("invalid robot model")
                        self.metadata[session] = metadata
                    except (ValueError, TypeError):
                        self.warnings["会话元数据损坏"] += 1
                    continue
                prefix = PREFIX.match(text)
                if not prefix:
                    continue
                clock, millis, level, tag, payload = prefix.groups()
                replay = REPLAY.match(payload)
                wall_ms = 0
                if wall_day:
                    parsed = dt.datetime.combine(wall_day, dt.time.fromisoformat(clock))
                    wall_ms = parsed.timestamp() * 1000 + int(millis)
                if not replay:
                    self.diagnostics.append({"session": session, "wall_ms": wall_ms,
                        "time": clock + "." + millis, "level": level, "tag": tag,
                        "text": unescape(payload)})
                    continue
                epoch, seq, payload = replay.groups()
                key = (session, int(seq))
                if key in self.seen:
                    continue
                self.seen.add(key)
                wall_ms = int(epoch) / 1e6
                wall_day = dt.datetime.fromtimestamp(wall_ms / 1000).date()
                if not payload.startswith("{"):
                    # Older LOG_TRAJECTORY and diagnostic records remain readable as text.
                    self.diagnostics.append({"session": session, "wall_ms": wall_ms,
                        "time": clock + "." + millis, "level": level, "tag": tag, "text": unescape(payload)})
                    continue
                try:
                    event = json.loads(unescape(payload), parse_constant=reject_constant)
                    if event.get("v") != 1:
                        self.warnings["不支持的回放协议版本"] += 1
                        continue
                    if not finite(event.get("stamp")):
                        raise ValueError("missing data timestamp")
                    if not isinstance(event.get("event"), str):
                        raise ValueError("missing event name")
                    for field in ("clock_epoch", "state_id", "map_id"):
                        if field in event and not isinstance(event[field], (int, str)):
                            raise ValueError("invalid ID")
                    if event.get("state") is not None:
                        pose = event["state"]["pose"]
                        if not vector(pose, 3):
                            raise ValueError("invalid pose")
                        if "velocity" in event["state"] and not vector(event["state"]["velocity"], 3):
                            raise ValueError("invalid velocity")
                    if event["event"] == "pose" and not isinstance(event.get("state"), dict):
                        raise ValueError("missing pose state")
                    for field in ("goal", "local_goal"):
                        if field in event and not vector(event[field], 3):
                            raise ValueError("invalid " + field)
                    if "command" in event and not vector(event["command"], 2):
                        raise ValueError("invalid command")
                    if event["event"] == "plan":
                        if (not isinstance(event.get("status"), str) or not finite(event.get("elapsed_ms")) or
                                not isinstance(event.get("success"), bool) or not isinstance(event.get("accepted"), bool) or
                                not vector(event.get("command"), 2) or
                                not all(k in event for k in ("plan_id", "state_id", "map_id"))):
                            raise ValueError("incomplete plan")
                        if "global_path" in event and (not isinstance(event["global_path"], list) or
                                not all(vector(p, 3) for p in event["global_path"])):
                            raise ValueError("invalid path")
                    if event["event"] == "costmap":
                        width, height = event["width"], event["height"]
                        if (not isinstance(width, int) or not isinstance(height, int) or
                                min(width, height) <= 0 or width * height > MAX_CELLS or
                                not finite(event["resolution"]) or event["resolution"] <= 0 or
                                len(event["origin"]) != 2 or not all(finite(x) for x in event["origin"])):
                            raise ValueError("invalid grid geometry")
                        if event.get("layout") != "row_major_y_x" or event.get("encoding") != "gzip+base64":
                            raise ValueError("unsupported grid encoding")
                        # Keep an on-disk index, not every expanded map in memory.
                        del event["data"]
                        event["_location"] = (str(path), offset)
                    event.update(session=session, seq=int(seq), wall_ms=wall_ms,
                                 wall_time=clock + "." + millis, epoch_ns=epoch)
                    self.records.append(event)
                except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
                    self.warnings["损坏或不完整的回放记录已跳过"] += 1

    def finalize(self):
        self.groups = {}
        self.maps = {}
        self.records.sort(key=lambda e: (e["wall_ms"], e["seq"]))
        for event in self.records:
            key = event["session"] + ":" + str(event.get("clock_epoch", 0))
            group = self.groups.setdefault(key, {"session": event["session"], "events": [], "plans": []})
            group["events"].append(event)
            if event["event"] == "plan":
                group["plans"].append(event)
            if event["event"] == "costmap":
                map_key = (event["session"], str(event["map_id"]))
                self.maps[map_key] = event
        for key, group in self.groups.items():
            group["events"].sort(key=lambda e: (e["stamp"], e["seq"]))
            group["plans"].sort(key=lambda e: (e["stamp"], e["seq"]))
            for kind in ("pose", "costmap", "plan", "goal", "command"):
                events = [e for e in group["events"] if e["event"] == kind and
                          not (kind == "costmap" and e.get("layer") == "global")]
                group[kind] = events
                group[kind + "_times"] = [e["stamp"] for e in events]
            group["times"] = [e["stamp"] for e in group["events"]]
        self.diagnostics.sort(key=lambda e: e["wall_ms"])
        self._diagnostics_by_session = {}
        for event in self.diagnostics:
            self._diagnostics_by_session.setdefault(event["session"], []).append(event)
        self._diagnostic_times = {s: [e["wall_ms"] for e in events]
                                  for s, events in self._diagnostics_by_session.items()}
        self._decoded.clear()

    def summary(self):
        groups = []
        for key, g in self.groups.items():
            events = g["events"]
            groups.append({"key": key, "session": g["session"], "frames": len(events),
                "plans": len(g["plans"]), "start": min(g["times"]), "end": max(g["times"]),
                "metadata": self.metadata.get(g["session"], {}),
                "dropped": max((e["dropped"] for e in events if e["event"] == "health" and finite(e.get("dropped"))), default=None),
                "errors": max((e["errors"] for e in events if e["event"] == "health" and finite(e.get("errors"))), default=None),
                "times": g["times"], "plan_times": [e["stamp"] for e in g["plans"]]})
        warnings = dict(self.warnings)
        if self.files and not groups:
            warnings["这些日志没有结构化回放数据；可查看文本，地图和定位需要重新录制"] = 1
        return {"files": self.files, "groups": groups, "warnings": warnings,
                "records": len(self.records), "text_lines": len(self.diagnostics),
                "text_preview": self.diagnostics[-100:] if not groups else []}

    @staticmethod
    def latest(group, kind, stamp):
        index = bisect.bisect_right(group[kind + "_times"], stamp) - 1
        return group[kind][index] if index >= 0 else None

    def frame(self, key, index, mode="plans"):
        group = self.groups[key]
        events = group["plans"] if mode == "plans" else group["events"]
        if index < 0 or index >= len(events):
            raise ValueError("frame index out of range")
        event = events[index]
        stamp, session = event["stamp"], group["session"]
        plan = event if event["event"] == "plan" else self.latest(group, "plan", stamp)
        pose = self.latest(group, "pose", stamp)
        state = event.get("state") if event["event"] == "plan" else event.get("state") or (pose or {}).get("state")
        if event["event"] == "plan" and event.get("state_id") == 0:
            state = None
        warnings = []
        map_event = (self.maps.get((session, str(event.get("map_id")))) if event["event"] == "plan"
                     else event if event["event"] == "costmap" and event.get("layer") == "local"
                     else self.latest(group, "costmap", stamp))
        if event["event"] == "plan" and not map_event:
            warnings.append("本帧尚未收到感知地图，正在等待输入" if event.get("map_id") == 0 else
                            "该规划帧引用的地图未导入或已丢失；不会用邻近地图替代")
        goal = self.latest(group, "goal", stamp)
        # A goal change/cancel invalidates an older plan in continuous event playback.
        if mode != "plans" and goal and plan and goal.get("goal_id") != plan.get("goal_id"):
            plan = None
        result = {"index": index, "stamp": stamp, "wall_time": event["wall_time"],
            "event": event["event"], "state": state, "goal": (goal or {}).get("goal"),
            "metadata": self.metadata.get(session, {}), "warnings": warnings,
            "maps": [], "plan": None, "command": self.latest(group, "command", stamp)}
        for m in (self.maps.get((session, "global")), map_event):
            if m:
                result["maps"].append({k: v for k, v in m.items() if not k.startswith("_")})
        if plan:
            result["goal"] = plan.get("goal")
            result["plan"] = {k: v for k, v in plan.items() if k not in ("trajectory", "initial_trajectory")}
            try:
                result["plan"]["trajectory"] = sample_trajectory(plan.get("trajectory"))
            except (ValueError, TypeError, KeyError, OverflowError):
                result["plan"]["trajectory"] = []
                warnings.append("轨迹系数损坏，已跳过该图层")
        pose_index = bisect.bisect_right(group["pose_times"], stamp)
        result["trace"] = [e["state"]["pose"] for e in group["pose"][max(0, pose_index-1000):pose_index]]
        diag = self._diagnostics_by_session.get(session, [])
        i = bisect.bisect_right(self._diagnostic_times.get(session, []), event["wall_ms"])
        result["logs"] = diag[max(0, i-80):i]
        return result

    def map_bytes(self, session, map_id):
        key = (session, str(map_id))
        if key in self._decoded:
            self._decoded.move_to_end(key)
            return self._decoded[key]
        event = self.maps[key]
        path, offset = event["_location"]
        with open(path, "rb") as stream:
            stream.seek(offset)
            raw = stream.readline(MAX_LINE + 1).decode("utf-8").rstrip("\r\n")
        payload = REPLAY.match(PREFIX.match(raw)[5])[3]
        fields = json.loads(unescape(payload))
        encoded = base64.b64decode(fields["data"], validate=True)
        expected = event["width"] * event["height"]
        with gzip.GzipFile(fileobj=io.BytesIO(encoded)) as stream:
            data = stream.read(expected + 1)
        if len(data) != expected:
            raise ValueError("解压后的栅格数量与地图宽高不一致")
        self._decoded[key] = data
        while len(self._decoded) > 8:
            self._decoded.popitem(last=False)
        return data
