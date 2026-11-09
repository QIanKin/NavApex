"use strict";
const $ = (id) => document.getElementById(id);
const canvas = $("map"),
  ctx = canvas.getContext("2d");
const layers = Object.fromEntries(
  [...document.querySelectorAll("[data-layer]")].map((e) => [
    e.dataset.layer,
    e.checked,
  ]),
);
const S = {
  summary: null,
  group: null,
  frame: null,
  index: 0,
  mode: "plans",
  times: [],
  playing: false,
  camera: { x: 0, y: 0, scale: 70 },
  maps: new Map(),
  generation: 0,
  loading: false,
  importing: false,
  importTask: false,
  logLines: [],
  fitted: false,
};
let noticeTimer;
function notice(text, persistent = false) {
  $("notice").textContent = text;
  $("notice").hidden = !text;
  clearTimeout(noticeTimer);
  if (!persistent)
    noticeTimer = setTimeout(() => ($("notice").hidden = true), 8000);
}
async function api(path, options) {
  const response = await fetch(path, options);
  if (!response.ok) {
    let error;
    try {
      error = (await response.json()).error;
    } catch {}
    throw Error(error || response.statusText);
  }
  return response.json();
}
function duration(seconds) {
  seconds = Math.max(0, seconds || 0);
  const minutes = Math.floor(seconds / 60),
    rest = (seconds % 60).toFixed(3).padStart(6, "0");
  return `${String(minutes).padStart(2, "0")}:${rest}`;
}
function upperBound(a, v) {
  let l = 0,
    r = a.length;
  while (l < r) {
    const m = (l + r) >> 1;
    if (a[m] <= v) l = m + 1;
    else r = m;
  }
  return l;
}
async function refresh() {
  const generation = S.generation;
  const summary = await api("/api/summary");
  if (S.importing || generation !== S.generation) return;
  S.summary = summary;
  $("sessions").replaceChildren();
  for (const [i, g] of S.summary.groups.entries()) {
    const option = new Option(
      `录制 ${i + 1} · ${g.plans} 规划 / ${g.frames} 事件`,
      g.key,
    );
    $("sessions").add(option);
  }
  $("fileStatus").textContent = S.summary.files.length
    ? `${S.summary.files.length} 个文件 · ${S.summary.records.toLocaleString()} 条回放记录`
    : "本地解析 · 数据保留在本机";
  const warnings = Object.entries(S.summary.warnings).map(
    ([k, v]) => `${k} (${v})`,
  );
  if (warnings.length) notice(warnings.join("\n"), true);
  if (S.summary.groups.length) await selectGroup();
  else {
    S.group = null;
    S.frame = null;
    S.times = [];
    S.logLines = S.summary.text_preview;
    renderLogs();
    draw();
    $("empty").hidden = false;
    $("play").disabled = true;
    $("seek").disabled = true;
    $("status").textContent = S.summary.files.length
      ? "仅文本日志 · 无结构化回放数据"
      : "等待导入日志";
    $("details").replaceChildren();
    $("wallClock").textContent = "--:--:--.---";
    $("frameLabel").textContent = "无回放帧";
    $("health").textContent = `文本日志 ${S.summary.text_lines} 行`;
  }
}
async function selectGroup() {
  if (S.importing) return;
  pause();
  S.generation++;
  S.group = S.summary.groups.find((g) => g.key === $("sessions").value);
  if (!S.group) return;
  S.maps.clear();
  S.fitted = false;
  if (!S.group.plans) $("mode").value = "events";
  S.mode = $("mode").value;
  S.times = S.mode === "plans" ? S.group.plan_times : S.group.times;
  if (!S.times.length) {
    $("mode").value = "events";
    S.mode = "events";
    S.times = S.group.times;
  }
  $("seek").max = Math.max(0, S.times.length - 1);
  $("seek").disabled = !S.times.length;
  $("play").disabled = !S.times.length;
  $("empty").hidden = true;
  $("health").textContent =
    `${S.group.metadata.frame || "未标记坐标系"} · 丢弃 ${S.group.dropped ?? "未知"} · 日志错误 ${S.group.errors ?? "未知"} · ${S.summary.files.length} 个文件`;
  if (S.group.dropped || S.group.errors)
    notice("记录期间发生日志丢弃或错误，请结合缺帧提示查看。", true);
  await showFrame(0);
}
function color(value, local) {
  if (value === 254) return [222, 232, 240, 255];
  if (value === 253) return [195, 100, 230, 245];
  if (value === 255) return [98, 107, 128, 200];
  if (value === 0) return local ? [33, 61, 67, 90] : [35, 46, 60, 255];
  const ratio = value / 252;
  return [
    Math.round(54 + ratio * 119),
    Math.round(83 + ratio * 33),
    Math.round(116 + ratio * 109),
    Math.round(75 + ratio * 150),
  ];
}
async function loadMap(map) {
  const key = map.session + ":" + map.map_id;
  if (S.maps.has(key)) {
    const cached = S.maps.get(key);
    S.maps.delete(key);
    S.maps.set(key, cached);
    return cached;
  }
  const response = await fetch(
    "/api/map?" + new URLSearchParams({ session: map.session, id: map.map_id }),
  );
  if (!response.ok) {
    const e = await response.json();
    throw Error(e.error || "地图解码失败");
  }
  const bytes = new Uint8Array(await response.arrayBuffer());
  if (bytes.length !== map.width * map.height)
    throw Error("地图栅格数量不匹配");
  const image = document.createElement("canvas");
  image.width = map.width;
  image.height = map.height;
  const gc = image.getContext("2d"),
    pixels = gc.createImageData(map.width, map.height);
  for (let i = 0; i < bytes.length; i++)
    pixels.data.set(color(bytes[i], map.layer === "local"), i * 4);
  gc.putImageData(pixels, 0, 0);
  const result = { ...map, image, bytes };
  S.maps.set(key, result);
  while (S.maps.size > 8) S.maps.delete(S.maps.keys().next().value);
  return result;
}
async function showFrame(index) {
  if (S.importing || !S.group || !S.times.length) return;
  index = Math.max(0, Math.min(S.times.length - 1, index));
  const token = ++S.generation;
  S.loading = true;
  try {
    const frame = await api(
      "/api/frame?" +
        new URLSearchParams({ session: S.group.key, index, mode: S.mode }),
    );
    const maps = await Promise.all(
      frame.maps.map(async (map) => {
        try {
          return await loadMap(map);
        } catch (e) {
          frame.warnings.push(e.message);
          return null;
        }
      }),
    );
    if (token !== S.generation) return;
    frame.loadedMaps = maps.filter(Boolean);
    S.frame = frame;
    S.index = index;
    $("seek").value = index;
    if (!S.fitted) {
      fitMap();
      S.fitted = true;
    }
    if ($("follow").checked && frame.state) {
      S.camera.x = frame.state.pose[0];
      S.camera.y = frame.state.pose[1];
    }
    renderDetails();
    draw();
  } catch (error) {
    if (token === S.generation) {
      pause();
      notice(error.message, true);
    }
  } finally {
    if (token === S.generation) S.loading = false;
  }
}
function renderDetails() {
  const f = S.frame,
    p = f.plan,
    st = f.state;
  $("wallClock").textContent = f.wall_time;
  $("sourceClock").textContent =
    `${f.metadata.source_clock || "数据"} ${f.stamp.toFixed(3)} s · +${duration(f.stamp - S.times[0])}`;
  $("frameLabel").textContent =
    `${S.mode === "plans" ? "规划帧" : "事件"} ${S.index + 1} / ${S.times.length}`;
  $("rangeLabel").textContent =
    `${duration(f.stamp - S.times[0])} / ${duration(S.times.at(-1) - S.times[0])}`;
  $("eventBadge").textContent = f.event;
  $("status").textContent =
    f.event === "perception"
      ? "感知处理失败 · 未生成新地图"
      : f.event === "costmap"
        ? "感知地图更新"
        : p
          ? p.status
          : "定位 / 感知回放";
  $("status").classList.toggle(
    "failed",
    f.event === "perception" || (!!p && (!p.success || !p.accepted)),
  );
  const pairs = [];
  if (st) {
    pairs.push(
      ["位置 x / y", `${st.pose[0].toFixed(3)} / ${st.pose[1].toFixed(3)} m`],
      ["朝向 yaw", `${((st.pose[2] * 180) / Math.PI).toFixed(1)}°`],
    );
    if (st.velocity)
      pairs.push([
        "实际速度",
        `${st.velocity[0].toFixed(3)} m/s · ${st.velocity[2].toFixed(3)} rad/s`,
      ]);
  }
  const robot = f.metadata.robot;
  if (robot)
    pairs.push([
      "Footprint",
      `${robot.shape} · R ${robot.radius.toFixed(3)} m`,
    ]);
  const local = f.maps.find((m) => m.layer === "local");
  if (local)
    pairs.push(
      ["局部地图", `${local.width}×${local.height} · ${local.resolution} m`],
      ["地图帧", String(local.map_id)],
    );
  if (p)
    pairs.push(
      ["规划 / 状态帧", `${p.plan_id} / ${p.state_id}`],
      ["规划引用地图", String(p.map_id)],
      ["结果采用", p.accepted ? "是" : "否"],
      ["耗时", `${p.elapsed_ms.toFixed(2)} ms`],
      ["规划指令", `${p.command[0].toFixed(3)} / ${p.command[1].toFixed(3)}`],
    );
  if (f.command)
    pairs.push([
      "已发布指令",
      f.command.command.map((x) => x.toFixed(3)).join(" / "),
    ]);
  $("details").replaceChildren();
  for (const [k, v] of pairs) {
    const dt = document.createElement("dt"),
      dd = document.createElement("dd");
    dt.textContent = k;
    dd.textContent = v;
    $("details").append(dt, dd);
  }
  $("mapWarning").textContent = f.warnings.join("；");
  $("mapWarning").hidden = !f.warnings.length;
  S.logLines = f.logs;
  renderLogs();
}
function renderLogs() {
  const filter = $("filter").value.toLowerCase();
  const lines = S.logLines.filter((l) =>
    `${l.level} ${l.tag} ${l.text}`.toLowerCase().includes(filter),
  );
  $("logs").replaceChildren();
  $("logCount").textContent = lines.length;
  for (const l of lines) {
    const row = document.createElement("div");
    row.className = "log-line " + l.level;
    const meta = document.createElement("div");
    meta.className = "meta";
    meta.textContent = `${l.time} ${l.level}/${l.tag}`;
    const message = document.createElement("div");
    message.textContent = l.text;
    row.append(meta, message);
    $("logs").append(row);
  }
  $("logs").scrollTop = $("logs").scrollHeight;
}
function fitMap() {
  const maps = S.frame?.loadedMaps;
  if (!maps?.length) return;
  const map = maps.find((m) => m.layer === "global") || maps[0],
    w = map.width * map.resolution,
    h = map.height * map.resolution;
  S.camera = {
    x: map.origin[0] + w / 2,
    y: map.origin[1] + h / 2,
    scale: Math.min(
      canvas.clientWidth / (w + 0.5),
      canvas.clientHeight / (h + 0.5),
    ),
  };
}
function world(x, y) {
  return [
    (x - canvas.clientWidth / 2) / S.camera.scale + S.camera.x,
    (canvas.clientHeight / 2 - y) / S.camera.scale + S.camera.y,
  ];
}
function line(points, color, width = 2, dashed = false) {
  if (!points?.length) return;
  ctx.beginPath();
  ctx.strokeStyle = color;
  ctx.lineWidth = width / S.camera.scale;
  ctx.setLineDash(dashed ? [5 / S.camera.scale, 4 / S.camera.scale] : []);
  points.forEach((p, i) =>
    i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]),
  );
  ctx.stroke();
  ctx.setLineDash([]);
}
function draw() {
  const ratio = window.devicePixelRatio || 1,
    w = canvas.clientWidth,
    h = canvas.clientHeight;
  if (
    canvas.width !== Math.round(w * ratio) ||
    canvas.height !== Math.round(h * ratio)
  ) {
    canvas.width = Math.round(w * ratio);
    canvas.height = Math.round(h * ratio);
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.translate(w / 2, h / 2);
  ctx.scale(S.camera.scale, -S.camera.scale);
  ctx.translate(-S.camera.x, -S.camera.y);
  const f = S.frame;
  const gridStep = S.camera.scale < 25 ? 2 : S.camera.scale > 180 ? 0.1 : 0.5;
  const [left, top] = world(0, 0),
    [right, bottom] = world(w, h);
  ctx.beginPath();
  ctx.strokeStyle = "#293646";
  ctx.lineWidth = 0.5 / S.camera.scale;
  for (
    let x = Math.ceil(left / gridStep) * gridStep;
    x < right;
    x += gridStep
  ) {
    ctx.moveTo(x, bottom);
    ctx.lineTo(x, top);
  }
  for (
    let y = Math.ceil(bottom / gridStep) * gridStep;
    y < top;
    y += gridStep
  ) {
    ctx.moveTo(left, y);
    ctx.lineTo(right, y);
  }
  ctx.stroke();
  if (f) {
    ctx.imageSmoothingEnabled = false;
    for (const map of f.loadedMaps || [])
      if (layers[map.layer]) {
        ctx.drawImage(
          map.image,
          map.origin[0],
          map.origin[1],
          map.width * map.resolution,
          map.height * map.resolution,
        );
        if (map.layer === "local") {
          ctx.strokeStyle = "#839ea866";
          ctx.lineWidth = 1 / S.camera.scale;
          ctx.strokeRect(
            map.origin[0],
            map.origin[1],
            map.width * map.resolution,
            map.height * map.resolution,
          );
        }
      }
    if (layers.trace) line(f.trace, "#9baab899", 1.5);
    const p = f.plan;
    if (p) {
      if (layers.globalPath) line(p.global_path, "#5a9cfb", 2);
      if (layers.trajectory)
        line(
          p.trajectory,
          p.accepted && p.success ? "#62e1cf" : "#ec8f78",
          2.5,
          !p.accepted || !p.success,
        );
    }
    if (f.goal) {
      ctx.strokeStyle = "#f6bc6b";
      ctx.lineWidth = 2 / S.camera.scale;
      const [x, y] = f.goal,
        r = 7 / S.camera.scale;
      ctx.beginPath();
      ctx.moveTo(x - r, y);
      ctx.lineTo(x + r, y);
      ctx.moveTo(x, y - r);
      ctx.lineTo(x, y + r);
      ctx.stroke();
    }
    if (layers.robot && f.state) {
      const [x, y, yaw] = f.state.pose,
        r = f.metadata.robot?.radius;
      if (Number.isFinite(r) && r > 0) {
        ctx.fillStyle = "#62e1cf25";
        ctx.strokeStyle = "#62e1cf";
        ctx.lineWidth = 2 / S.camera.scale;
        ctx.beginPath();
        ctx.arc(x, y, r, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }
      const arrow = Number.isFinite(r) ? r : 0.12;
      ctx.strokeStyle = "#b8fff4";
      ctx.lineWidth = 2 / S.camera.scale;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(
        x + Math.cos(yaw) * arrow * 1.5,
        y + Math.sin(yaw) * arrow * 1.5,
      );
      ctx.stroke();
      ctx.fillStyle = "#b8fff4";
      ctx.beginPath();
      ctx.arc(x, y, 2.5 / S.camera.scale, 0, Math.PI * 2);
      ctx.fill();
    }
  }
  const scaleMeters = 10 ** Math.floor(Math.log10(100 / S.camera.scale));
  $("scale").textContent =
    `${scaleMeters} m = ${Math.round(scaleMeters * S.camera.scale)} px`;
}
function pause() {
  S.playing = false;
  $("play").textContent = "播放";
}
function play() {
  if (S.importing || !S.times.length) return;
  if (S.playing) {
    pause();
    return;
  }
  const index = S.index >= S.times.length - 1 ? 0 : S.index;
  if (index !== S.index) showFrame(index);
  S.playing = true;
  $("play").textContent = "暂停";
  S.playStart = performance.now();
  S.playStamp = S.times[index];
  requestAnimationFrame(tick);
}
function tick(now) {
  if (!S.playing) return;
  const stamp =
      S.playStamp + ((now - S.playStart) / 1000) * Number($("speed").value),
    index = Math.min(
      S.times.length - 1,
      Math.max(0, upperBound(S.times, stamp) - 1),
    );
  if (index !== S.index && !S.loading) showFrame(index);
  if (stamp >= S.times.at(-1) && S.index === S.times.length - 1) {
    pause();
    return;
  }
  requestAnimationFrame(tick);
}
async function importFiles(files) {
  if (!files.length || S.importTask) return;
  pause();
  S.generation++;
  S.importing = true;
  S.importTask = true;
  for (const id of [
    "files",
    "emptyFiles",
    "mode",
    "sessions",
    "seek",
    "play",
    "prev",
    "next",
  ])
    $(id).disabled = true;
  notice("正在导入并建立索引…", true);
  try {
    for (let i = 0; i < files.length; i++) {
      notice(`正在导入 ${i + 1}/${files.length} · ${files[i].name}`, true);
      await api(
        "/api/import?" +
          new URLSearchParams({
            name: files[i].name,
            reset: i === 0 ? "1" : "0",
          }),
        {
          method: "POST",
          headers: { "Content-Type": "application/octet-stream" },
          body: files[i],
        },
      );
    }
    S.maps.clear();
    S.importing = false;
    await refresh();
    if (!Object.keys(S.summary.warnings).length) notice("导入完成");
  } catch (error) {
    S.importing = false;
    await refresh().catch(() => {});
    notice(error.message, true);
  } finally {
    S.importing = false;
    S.importTask = false;
    for (const id of [
      "files",
      "emptyFiles",
      "mode",
      "sessions",
      "prev",
      "next",
    ])
      $(id).disabled = false;
    $("files").value = "";
    $("emptyFiles").value = "";
  }
}
$("files").onchange = (e) => importFiles([...e.target.files]);
$("emptyFiles").onchange = (e) => importFiles([...e.target.files]);
$("sessions").onchange = () => selectGroup().catch((e) => notice(e.message));
$("mode").onchange = () => selectGroup().catch((e) => notice(e.message));
$("play").onclick = play;
$("prev").onclick = () => {
  pause();
  showFrame(S.index - 1);
};
$("next").onclick = () => {
  pause();
  showFrame(S.index + 1);
};
$("seek").oninput = (e) => {
  pause();
  showFrame(Number(e.target.value));
};
$("speed").onchange = () => {
  if (S.playing) {
    pause();
    play();
  }
};
$("fit").onclick = () => {
  $("follow").checked = false;
  fitMap();
  draw();
};
$("filter").oninput = renderLogs;
document.querySelectorAll("[data-layer]").forEach(
  (el) =>
    (el.onchange = () => {
      layers[el.dataset.layer] = el.checked;
      draw();
    }),
);
let dragging = null;
canvas.onpointerdown = (e) => {
  dragging = { x: e.clientX, y: e.clientY, camera: { ...S.camera } };
  canvas.setPointerCapture(e.pointerId);
  $("follow").checked = false;
};
canvas.onpointerup = canvas.onpointercancel = () => (dragging = null);
canvas.onpointermove = (e) => {
  if (dragging) {
    S.camera.x = dragging.camera.x - (e.clientX - dragging.x) / S.camera.scale;
    S.camera.y = dragging.camera.y + (e.clientY - dragging.y) / S.camera.scale;
    draw();
  }
  const rect = canvas.getBoundingClientRect(),
    [x, y] = world(e.clientX - rect.left, e.clientY - rect.top);
  let text = `x ${x.toFixed(3)} · y ${y.toFixed(3)} m`;
  for (const m of [...(S.frame?.loadedMaps || [])].reverse()) {
    if (!layers[m.layer]) continue;
    const col = Math.floor((x - m.origin[0]) / m.resolution),
      row = Math.floor((y - m.origin[1]) / m.resolution);
    if (col >= 0 && row >= 0 && col < m.width && row < m.height) {
      text += ` · cost ${m.bytes[row * m.width + col]} [${col},${row}]`;
      break;
    }
  }
  $("coordinates").textContent = text;
};
canvas.addEventListener(
  "wheel",
  (e) => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect(),
      x = e.clientX - rect.left,
      y = e.clientY - rect.top,
      a = world(x, y);
    S.camera.scale = Math.min(
      3000,
      Math.max(3, S.camera.scale * Math.exp(-e.deltaY * 0.001)),
    );
    const b = world(x, y);
    S.camera.x += a[0] - b[0];
    S.camera.y += a[1] - b[1];
    draw();
  },
  { passive: false },
);
new ResizeObserver(() => draw()).observe($("stage"));
document.addEventListener("keydown", (e) => {
  if (["INPUT", "SELECT", "TEXTAREA"].includes(e.target.tagName)) return;
  if (e.code === "Space") {
    e.preventDefault();
    play();
  }
  if (e.code === "ArrowLeft") $("prev").click();
  if (e.code === "ArrowRight") $("next").click();
});
document.addEventListener("dragover", (e) => {
  e.preventDefault();
  $("dragHint").hidden = false;
});
document.addEventListener("dragleave", (e) => {
  if (!e.relatedTarget) $("dragHint").hidden = true;
});
document.addEventListener("drop", (e) => {
  e.preventDefault();
  $("dragHint").hidden = true;
  importFiles([...e.dataTransfer.files]);
});
refresh().catch((e) => notice(e.message, true));
