/* SLAM 可视化演示站前端（无框架，Canvas 2D，rviz 深色工程风）
 * 数据契约：assets/data/CONTRACT.md（v3：replay_index 多场景段 / slam_replay/2 对照链 / icp_grid 收敛域）
 * 加载顺序：assets/data/<name>.json → assets/data/__fixture__/<name>.json → 占位提示
 * 正式文件一旦出现在 assets/data/ 根下即自动优先于 fixture，无需改代码。
 */
(function () {
  "use strict";

  /* ================= 通用工具 ================= */

  function $(id) { return document.getElementById(id); }

  // 依次尝试正式路径与 fixture；返回 { data, source } 或 null
  function loadData(name) {
    var bases = ["assets/data/", "assets/data/__fixture__/"];
    var labels = ["正式数据", "示例小样本"];
    var chain = Promise.reject(new Error("start"));
    bases.forEach(function (b, i) {
      chain = chain.catch(function () {
        return fetch(b + name).then(function (r) {
          if (!r.ok) throw new Error("404");
          return r.json().then(function (j) { return { data: j, source: labels[i] }; });
        });
      });
    });
    return chain.catch(function () { return null; });
  }

  // 只探测可用性（HEAD，不下载正文），用于段选择器灰显；返回来源标签或 null
  function probeData(name) {
    var bases = ["assets/data/", "assets/data/__fixture__/"];
    var labels = ["正式数据", "示例小样本"];
    var chain = Promise.reject(new Error("start"));
    bases.forEach(function (b, i) {
      chain = chain.catch(function () {
        return fetch(b + name, { method: "HEAD" }).then(function (r) {
          if (!r.ok) throw new Error("404");
          return labels[i];
        });
      });
    });
    return chain.catch(function () { return null; });
  }

  function srcNote(source) {
    return source ? "数据来源：" + source : "数据尚未生成（将由 tools 导出）";
  }

  function wrapAngle(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  // 深色 rviz 风等比例坐标系绘图器：自适应 DPR 与容器宽度
  var C = {
    grid: "#242a34", gridText: "#5c6675", axis: "#3a4250", axisText: "#7c8698",
    map: "#6f8265", scan: "#ffc145", traj: "#4da3ff", robot: "#ff5f57", alt: "#ff9f43"
  };

  function makePlot(canvas) {
    var ctx = canvas.getContext("2d");
    var view = { cx: 0, cy: 0, scale: 50 }; // scale: px / m
    function resize() {
      var w = canvas.clientWidth, h = canvas.clientHeight;
      var dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.round(w * dpr));
      canvas.height = Math.max(1, Math.round(h * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
    resize();
    var plot = {
      ctx: ctx,
      resize: resize,
      width: function () { return canvas.clientWidth; },
      height: function () { return canvas.clientHeight; },
      // 依据点集计算等比例视图（含边距）
      fit: function (pointSets, padFrac) {
        var minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
        pointSets.forEach(function (pts) {
          for (var i = 0; i < pts.length; i++) {
            var p = pts[i];
            if (p[0] < minX) minX = p[0]; if (p[0] > maxX) maxX = p[0];
            if (p[1] < minY) minY = p[1]; if (p[1] > maxY) maxY = p[1];
          }
        });
        if (!isFinite(minX)) { minX = -1; minY = -1; maxX = 1; maxY = 1; }
        plot.fitBounds([minX, minY, maxX, maxY], padFrac);
      },
      // 直接给定数据包围盒（调用方预算好，避免每次遍历全量地图点）
      fitBounds: function (bb, padFrac) {
        var pad = padFrac == null ? 0.08 : padFrac;
        var dx = (bb[2] - bb[0]) || 1, dy = (bb[3] - bb[1]) || 1;
        var w = canvas.clientWidth, h = canvas.clientHeight;
        view.scale = Math.min(w / (dx * (1 + pad * 2)), h / (dy * (1 + pad * 2)));
        view.cx = (bb[0] + bb[2]) / 2;
        view.cy = (bb[1] + bb[3]) / 2;
      },
      toScreen: function (x, y) {
        // 数据 y 向上为正 → 屏幕 y 向下
        return [
          canvas.clientWidth / 2 + (x - view.cx) * view.scale,
          canvas.clientHeight / 2 - (y - view.cy) * view.scale
        ];
      },
      clear: function () {
        ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
      },
      // 米制网格 + 坐标轴（x 前 / y 左），深色
      grid: function () {
        var w = canvas.clientWidth, h = canvas.clientHeight;
        var halfWm = w / 2 / view.scale, halfHm = h / 2 / view.scale;
        var span = Math.max(halfWm, halfHm) * 2;
        var steps = [0.5, 1, 2, 5, 10, 20, 50, 100, 200];
        var step = steps[steps.length - 1];
        for (var i = 0; i < steps.length; i++) {
          if (span / steps[i] <= 12) { step = steps[i]; break; }
        }
        var x0 = Math.floor((view.cx - halfWm) / step) * step;
        var x1 = view.cx + halfWm, y0 = Math.floor((view.cy - halfHm) / step) * step, y1 = view.cy + halfHm;
        ctx.strokeStyle = C.grid; ctx.lineWidth = 1;
        ctx.font = "11px Consolas, monospace"; ctx.fillStyle = C.gridText;
        var gx, gy, s;
        for (gx = x0; gx <= x1; gx += step) {
          s = plot.toScreen(gx, 0);
          ctx.beginPath(); ctx.moveTo(s[0], 0); ctx.lineTo(s[0], h); ctx.stroke();
          ctx.fillText(trimNum(gx), s[0] + 3, h - 6);
        }
        for (gy = y0; gy <= y1; gy += step) {
          s = plot.toScreen(0, gy);
          ctx.beginPath(); ctx.moveTo(0, s[1]); ctx.lineTo(w, s[1]); ctx.stroke();
          ctx.fillText(trimNum(gy), 4, s[1] - 3);
        }
        // 坐标轴
        var o = plot.toScreen(0, 0);
        ctx.strokeStyle = C.axis; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(0, o[1]); ctx.lineTo(w, o[1]); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(o[0], 0); ctx.lineTo(o[0], h); ctx.stroke();
        ctx.fillStyle = C.axisText; ctx.font = "12px sans-serif";
        ctx.fillText("x (前)", w - 46, o[1] - 6);
        ctx.fillText("y (左)", o[0] + 6, 14);
      },
      points: function (pts, color, radius) {
        plot.pointsRange(pts, 0, pts.length, color, radius);
      },
      // 只画 [start, end) 区间；单路径批量提交，4 万点也不卡
      pointsRange: function (pts, start, end, color, radius) {
        var r = radius || 2;
        ctx.fillStyle = color;
        ctx.beginPath();
        for (var i = start; i < end && i < pts.length; i++) {
          var s = plot.toScreen(pts[i][0], pts[i][1]);
          ctx.moveTo(s[0] + r, s[1]);
          ctx.arc(s[0], s[1], r, 0, 6.2832);
        }
        ctx.fill();
      },
      polyline: function (pts, color, widthPx) {
        if (!pts.length) return;
        ctx.strokeStyle = color; ctx.lineWidth = widthPx || 2;
        ctx.beginPath();
        var s = plot.toScreen(pts[0][0], pts[0][1]);
        ctx.moveTo(s[0], s[1]);
        for (var i = 1; i < pts.length; i++) {
          s = plot.toScreen(pts[i][0], pts[i][1]);
          ctx.lineTo(s[0], s[1]);
        }
        ctx.stroke();
      }
    };
    return plot;
  }

  function trimNum(v) {
    return (Math.abs(v) < 1e-9 ? 0 : v).toString();
  }

  function transformPoints(src, R, t) {
    var out = new Array(src.length);
    for (var i = 0; i < src.length; i++) {
      var x = src[i][0], y = src[i][1];
      out[i] = [R[0][0] * x + R[0][1] * y + t[0], R[1][0] * x + R[1][1] * y + t[1]];
    }
    return out;
  }

  /* ================= 路由 ================= */

  var ROUTES = ["slam", "icp", "traj", "notes"];
  var inited = {}; // 每个标签页只初始化一次

  function currentRoute() {
    var h = location.hash.replace(/^#\/?/, "").split("/")[0];
    return ROUTES.indexOf(h) >= 0 ? h : "slam"; // 落地即 SLAM 回放页
  }

  function route() {
    var r = currentRoute();
    ROUTES.forEach(function (name) {
      $("page-" + name).classList.toggle("active", name === r);
    });
    document.querySelectorAll("#nav a").forEach(function (a) {
      a.classList.toggle("active", a.getAttribute("data-route") === r);
    });
    $("nav").classList.remove("open");
    $("nav-toggle").setAttribute("aria-expanded", "false");
    if (!inited[r]) {
      inited[r] = true;
      INIT[r]();
    } else if (REDRAW[r]) {
      // 回到标签页时画布尺寸可能变化，重绘一次
      REDRAW[r]();
    }
  }

  window.addEventListener("hashchange", route);
  $("nav-toggle").addEventListener("click", function () {
    var nav = $("nav");
    nav.classList.toggle("open");
    $("nav-toggle").setAttribute("aria-expanded", nav.classList.contains("open") ? "true" : "false");
  });

  /* ================= SLAM 建图回放 ================= */

  var slam = {
    data: null, idx: 0, playing: false, raf: 0, acc: 0, last: 0,
    plot: null, bbox: null, posesXY: [], pathCum: [0], rmseArr: [], inlierArr: [],
    segments: null, segId: null, reqSeq: 0,
    altPoses: null, altLabel: "", showAlt: false
  };

  function initSlam() {
    slam.plot = makePlot($("slam-canvas"));
    $("slam-play").addEventListener("click", function () {
      if (!slam.data) return;
      // 在最后一帧按播放 = 从头再看一遍
      if (!slam.playing && slam.idx >= slam.data.frames.length - 1) setSlamFrame(0);
      slam.playing = !slam.playing;
      $("slam-play").textContent = slam.playing ? "⏸ 暂停" : "▶ 播放";
      slam.last = 0; slam.acc = 0;
      if (slam.playing) slam.raf = requestAnimationFrame(slamTick);
    });
    $("slam-slider").addEventListener("input", function () {
      setSlamFrame(parseInt(this.value, 10) || 0);
    });
    $("slam-alt-toggle").addEventListener("change", function () {
      slam.showAlt = this.checked && !!slam.altPoses;
      refreshAltLegend();
      drawSlam();
    });
    window.addEventListener("resize", redrawSlam);
    // 先读多场景段索引（契约 v3 第 5 节）；索引缺失时退回 v2 单段行为
    loadData("replay_index.json").then(function (idxRes) {
      if (idxRes && idxRes.data && idxRes.data.segments && idxRes.data.segments.length) {
        setupSegments(idxRes.data.segments);
      } else {
        loadSlamSegment({ id: "main", label: "大厅段", file: "slam_replay.json" });
      }
    });
  }

  // 渲染段选择器按钮，逐段探测数据是否就绪（未就绪灰显）
  function setupSegments(segments) {
    slam.segments = segments;
    $("slam-seg-bar").hidden = false;
    var box = $("slam-segments");
    box.innerHTML = "";
    segments.forEach(function (seg) {
      var b = document.createElement("button");
      b.className = "init-btn seg-btn";
      b.textContent = seg.label || seg.id;
      b.setAttribute("data-seg", seg.id);
      b.title = seg.desc || "";
      b.addEventListener("click", function () {
        if (!b.disabled) selectSegment(seg.id);
      });
      box.appendChild(b);
    });
    var probes = segments.map(function (seg) {
      return probeData(seg.file).then(function (label) {
        var btn = box.querySelector('[data-seg="' + seg.id + '"]');
        if (!btn) return label;
        if (!label) {
          btn.disabled = true;
          btn.title = (seg.desc || seg.label || seg.id) + " —— 数据尚未生成";
        } else {
          btn.title = (seg.desc || seg.label || seg.id) +
            (seg.tags && seg.tags.length ? "（" + seg.tags.join(" / ") + "）" : "") +
            " —— " + label;
        }
        return label;
      });
    });
    Promise.all(probes).then(function (labels) {
      var def = null, i;
      for (i = 0; i < segments.length; i++) {           // 契约：id=main 为默认段
        if (segments[i].id === "main" && labels[i]) { def = segments[i]; break; }
      }
      for (i = 0; !def && i < segments.length; i++) {   // main 未就绪时取第一个可用段
        if (labels[i]) { def = segments[i]; break; }
      }
      if (def) {
        selectSegment(def.id);
      } else {
        $("slam-status").textContent = "所有场景段的数据都尚未生成（将由 tools 导出）";
        drawSlamPlaceholder();
      }
    });
  }

  function selectSegment(id) {
    var seg = null;
    (slam.segments || []).forEach(function (s) { if (s.id === id) seg = s; });
    if (!seg) return;
    slam.segId = id;
    document.querySelectorAll("#slam-segments .seg-btn").forEach(function (b) {
      b.classList.toggle("active", b.getAttribute("data-seg") === id);
    });
    loadSlamSegment(seg);
  }

  // 加载并重置一个场景段（切换段 = 重新加载该段 JSON + 回放归零）
  function loadSlamSegment(seg) {
    slam.playing = false;
    if (slam.raf) cancelAnimationFrame(slam.raf);
    $("slam-play").textContent = "▶ 播放";
    slam.data = null; slam.idx = 0;
    slam.posesXY = []; slam.pathCum = [0]; slam.rmseArr = []; slam.inlierArr = [];
    slam.altPoses = null; slam.altLabel = ""; slam.showAlt = false;
    $("slam-alt-wrap").hidden = true;      // 契约：/1 无 alt 时隐藏对照开关
    $("slam-alt-toggle").checked = false;
    $("stat-alt-row").hidden = true;
    refreshAltLegend();
    var slider = $("slam-slider");
    slider.max = "0"; slider.value = "0";
    var myReq = ++slam.reqSeq;             // 防止快速切换时旧响应覆盖新段
    $("slam-status").textContent = "正在加载「" + (seg.label || seg.id) + "」…";
    drawSlamPlaceholder();
    loadData(seg.file).then(function (res) {
      if (myReq !== slam.reqSeq) return;
      if (!res) {
        $("slam-status").textContent = "「" + (seg.label || seg.id) + "」数据尚未生成（将由 tools 导出 " + seg.file + "）";
        drawSlamPlaceholder();
        return;
      }
      slam.data = res.data;
      var frames = res.data.frames;
      // 预算：位姿序列、路径累计长度（相邻 pose 累加）、总包围盒
      var minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
      function eat(x, y) {
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
      }
      frames.forEach(function (f, i) {
        slam.posesXY.push([f.pose[0], f.pose[1]]);
        slam.rmseArr.push(f.rmse);
        slam.inlierArr.push(f.n_inlier);
        if (i > 0) {
          var p = frames[i - 1].pose;
          slam.pathCum.push(slam.pathCum[i - 1] + Math.hypot(f.pose[0] - p[0], f.pose[1] - p[1]));
        }
        eat(f.pose[0], f.pose[1]);
      });
      (res.data.map || []).forEach(function (p) { eat(p[0], p[1]); });
      // 对照链（schema slam_replay/2 可选字段）：零初值重跑的位姿链
      if (res.data.alt && res.data.alt.poses && res.data.alt.poses.length) {
        slam.altPoses = [];
        res.data.alt.poses.forEach(function (p) {
          slam.altPoses.push([p[0], p[1]]);
          eat(p[0], p[1]);
        });
        slam.altLabel = res.data.alt.label || "对照链";
        $("slam-alt-text").textContent = slam.altLabel;
        $("slam-alt-wrap").hidden = false;
        // 侧栏：双链末端偏差 |Δx|, |Δy| 与 Δθ
        var lf = frames[frames.length - 1].pose;
        var la = res.data.alt.poses[res.data.alt.poses.length - 1];
        $("stat-alt").textContent =
          "|Δx|=" + Math.abs(lf[0] - la[0]).toFixed(3) +
          ", |Δy|=" + Math.abs(lf[1] - la[1]).toFixed(3) +
          " m ｜ Δθ=" + Math.abs(wrapAngle(lf[2] - (la[2] || 0))).toFixed(4) + " rad";
        $("stat-alt-row").hidden = false;
      }
      slam.bbox = [minX, minY, maxX, maxY];
      slider.max = String(frames.length - 1);
      slider.value = "0";
      slam.idx = 0;
      var meta = res.data.meta || {};
      $("slam-status").textContent = "当前段：" + (seg.label || seg.id) +
        (seg.desc ? "（" + seg.desc + "）" : "") + "；" + srcNote(res.source) +
        "；数据集共 " + (meta.frames_total || frames.length) + " 帧" +
        (meta.method ? "；方法：" + meta.method : "") +
        (meta.path_length_m != null ? "；全程路径约 " + meta.path_length_m + " m" : "");
      drawSlam();
    });
  }

  // 图例：对照链开启时区分两条链（恒速初值 / 零初值）
  function refreshAltLegend() {
    var show = slam.showAlt && slam.altPoses;
    $("lg-alt-wrap").hidden = !show;
    $("lg-traj-text").textContent = show ? "轨迹·恒速初值" : "已走轨迹";
    if (show) $("lg-alt-text").textContent = slam.altLabel || "对照链·零初值";
  }

  function redrawSlam() {
    if (slam.data) drawSlam(); else drawSlamPlaceholder();
  }

  function setSlamFrame(i) {
    if (!slam.data) return;
    var n = slam.data.frames.length;
    slam.idx = Math.max(0, Math.min(n - 1, i));
    $("slam-slider").value = String(slam.idx);
    drawSlam();
  }

  function slamTick(now) {
    if (!slam.playing || !slam.data) return;
    var frames = slam.data.frames;
    var speed = parseFloat($("slam-speed").value) || 1;
    if (!slam.last) slam.last = now;
    slam.acc += (now - slam.last) / 1000 * speed;
    slam.last = now;
    // 用相邻帧真实时间戳差做播放间隔（契约 v2：t 为相对秒，首帧 t=0）
    var cur = frames[slam.idx];
    var next = frames[Math.min(slam.idx + 1, frames.length - 1)];
    var dt = Math.min(1, Math.max(0.01, (next.t - cur.t) || 0.041));
    if (slam.acc >= dt) {
      slam.acc = 0;
      if (slam.idx >= frames.length - 1) {
        slam.playing = false;
        $("slam-play").textContent = "▶ 播放";
      } else {
        setSlamFrame(slam.idx + 1);
      }
    }
    if (slam.playing) slam.raf = requestAnimationFrame(slamTick);
  }

  function drawSlam() {
    if (!slam.data) return;
    var d = slam.data, i = slam.idx, f = d.frames[i];
    var plot = slam.plot;
    plot.resize();
    plot.fitBounds(slam.bbox); // 固定视图：回放时地图不跳动
    plot.clear();
    plot.grid();
    // 累积地图点（暗灰绿）→ 已走轨迹（蓝）→ 对照链（橙）→ 当前帧扫描（亮黄）→ 机器人（红三角）
    plot.pointsRange(d.map, 0, f.map_upto, C.map, 1.7);
    plot.polyline(slam.posesXY.slice(0, i + 1), C.traj, 2);
    if (slam.showAlt && slam.altPoses) {
      plot.polyline(slam.altPoses.slice(0, Math.min(i + 1, slam.altPoses.length)), C.alt, 2);
    }
    plot.points(f.scan, C.scan, 2.2);
    drawRobot(plot, f.pose);
    // 信息行 + 状态面板 + 迷你曲线
    $("slam-info").textContent =
      "帧 " + f.i + " ｜ t = " + Number(f.t).toFixed(3) + " s ｜ " + (i + 1) + "/" + d.frames.length;
    $("stat-pose").textContent = "[" + f.pose.map(function (v) { return Number(v).toFixed(3); }).join(", ") + "]";
    $("stat-rmse").textContent = Number(f.rmse).toFixed(6) + " m";
    $("stat-inlier").textContent = String(f.n_inlier);
    $("stat-map").textContent = String(f.map_upto);
    $("stat-path").textContent = slam.pathCum[i].toFixed(2) + " m";
    drawSpark("spark-rmse", slam.rmseArr, i, "#f59e0b", function (v) { return v.toFixed(4); });
    drawSpark("spark-inlier", slam.inlierArr, i, "#4da3ff", function (v) { return String(Math.round(v)); });
  }

  function drawSlamPlaceholder() {
    var plot = slam.plot;
    plot.resize();
    plot.fit([[-5, -5], [5, 5]]);
    plot.clear();
    plot.grid();
    var ctx = plot.ctx;
    ctx.fillStyle = C.gridText; ctx.font = "14px sans-serif"; ctx.textAlign = "center";
    ctx.fillText("SLAM 回放数据尚未生成", plot.width() / 2, plot.height() / 2);
    ctx.textAlign = "left";
  }

  // 机器人：带朝向的三角（世界坐标 0.24m 尺寸，随缩放）
  function drawRobot(plot, pose) {
    var x = pose[0], y = pose[1], th = pose[2];
    var apex = [x + 0.24 * Math.cos(th), y + 0.24 * Math.sin(th)];
    var b1 = [x + 0.15 * Math.cos(th + 2.55), y + 0.15 * Math.sin(th + 2.55)];
    var b2 = [x + 0.15 * Math.cos(th - 2.55), y + 0.15 * Math.sin(th - 2.55)];
    var a = plot.toScreen(apex[0], apex[1]);
    var p1 = plot.toScreen(b1[0], b1[1]);
    var p2 = plot.toScreen(b2[0], b2[1]);
    var ctx = plot.ctx;
    ctx.fillStyle = C.robot;
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]); ctx.lineTo(p1[0], p1[1]); ctx.lineTo(p2[0], p2[1]);
    ctx.closePath(); ctx.fill();
    ctx.strokeStyle = "rgba(255,255,255,0.85)"; ctx.lineWidth = 1.2; ctx.stroke();
  }

  // 迷你曲线（sparkline）：整条序列折线 + 当前帧竖线标记
  function drawSpark(id, values, idx, color, fmt) {
    var canvas = $(id);
    if (!canvas) return;
    var w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    var dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(w * dpr));
    canvas.height = Math.max(1, Math.round(h * dpr));
    var ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    var lo = Math.min.apply(null, values), hi = Math.max.apply(null, values);
    if (hi <= lo) hi = lo + 1;
    var padL = 6, padR = 6, padT = 14, padB = 6;
    function X(i) { return padL + (w - padL - padR) * (values.length <= 1 ? 0.5 : i / (values.length - 1)); }
    function Y(v) { return padT + (h - padT - padB) * (1 - (v - lo) / (hi - lo)); }
    ctx.strokeStyle = "#242a34"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(padL, Y(hi)); ctx.lineTo(w - padR, Y(hi)); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(padL, Y(lo)); ctx.lineTo(w - padR, Y(lo)); ctx.stroke();
    ctx.fillStyle = "#5c6675"; ctx.font = "10px Consolas, monospace";
    ctx.fillText(fmt(hi), padL + 2, 10);
    ctx.fillText(fmt(lo), padL + 2, h - 2);
    ctx.strokeStyle = color; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(X(0), Y(values[0]));
    for (var i = 1; i < values.length; i++) ctx.lineTo(X(i), Y(values[i]));
    ctx.stroke();
    var mx = X(idx);
    ctx.strokeStyle = "rgba(255,255,255,0.5)"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(mx, 2); ctx.lineTo(mx, h - 2); ctx.stroke();
    ctx.fillStyle = "#ffffff";
    ctx.beginPath(); ctx.arc(mx, Y(values[idx]), 3, 0, 6.2832); ctx.fill();
  }

  /* ================= ICP 实验 ================= */

  var icp = {
    data: null, run: null, it: 0, playing: false, raf: 0, last: 0, plot: null, curve: null,
    traceData: null, traceStatus: "",          // icp_trace.json（初值扫描实验）
    gridData: null, gridDims: null, gridRange: null, gridRects: [], gridHover: -1, gridSel: -1
  };

  function initIcp() {
    icp.plot = makePlot($("icp-canvas"));
    icp.curve = $("icp-curve").getContext("2d");
    $("icp-play").addEventListener("click", function () {
      if (!icp.run) return;
      icp.playing = !icp.playing;
      $("icp-play").textContent = icp.playing ? "⏸ 暂停" : "▶ 播放迭代";
      icp.last = 0;
      if (icp.playing) icp.raf = requestAnimationFrame(icpTick);
    });
    $("icp-slider").addEventListener("input", function () {
      icp.it = parseInt(this.value, 10) || 0;
      drawIcp();
    });
    var gc = $("icp-grid");
    gc.addEventListener("click", onGridPick);
    gc.addEventListener("mousemove", onGridHover);
    gc.addEventListener("mouseleave", function () {
      icp.gridHover = -1;
      $("icp-grid-info").textContent = "";
      drawIcpGrid();
    });
    window.addEventListener("resize", function () { drawIcp(); drawIcpGrid(); });
    loadData("icp_trace.json").then(function (res) {
      if (!res) {
        $("icp-status").textContent = srcNote(null);
        return;
      }
      icp.traceData = res.data;
      icp.data = res.data;
      var box = $("icp-buttons");
      box.innerHTML = "";
      res.data.runs.forEach(function (run, i) {
        var b = document.createElement("button");
        b.className = "init-btn";
        b.textContent = "初值 " + run.init_deg + "°";
        b.addEventListener("click", function () { selectRun(i); });
        box.appendChild(b);
      });
      icp.traceStatus = srcNote(res.source) +
        "；真值（读自数据文件）：旋转 " + res.data.truth.deg + "°，平移 [" +
        res.data.truth.t.map(function (v) { return Number(v).toFixed(4); }).join(", ") + "] m";
      $("icp-status").textContent = icp.traceStatus;
      if (!icp.run) selectRun(0); // 用户可能已从地形图选格，别覆盖
    });
    // 收敛域地形图（契约 v3 第 7 节）
    loadData("icp_grid.json").then(function (res) {
      if (!res) {
        $("icp-grid-status").textContent = "收敛域地形图数据尚未生成（将由 tools 导出 icp_grid.json）";
        drawGridPlaceholder();
        return;
      }
      icp.gridData = res.data;
      buildGridIndex();
      drawIcpGrid();
      var g = res.data.grid;
      $("icp-grid-status").textContent = srcNote(res.source) +
        "；网格 " + g.degs.length + "×" + g.dxs.length + " = " + (res.data.cells || []).length +
        " 格 ｜ 真值：旋转 " + res.data.truth.deg + "°，平移 [" +
        res.data.truth.t.map(function (v) { return Number(v).toFixed(4); }).join(", ") + "] m";
    });
  }

  // 初值按钮：数据源切回 icp_trace 的 run
  function selectRun(i) {
    icp.data = icp.traceData;
    icp.run = icp.traceData.runs[i];
    icp.gridSel = -1;
    icp.it = 0;
    icp.playing = false;
    $("icp-play").textContent = "▶ 播放迭代";
    var slider = $("icp-slider");
    slider.max = String(icp.run.iterations.length - 1);
    slider.value = "0";
    document.querySelectorAll("#icp-buttons .init-btn").forEach(function (b, j) {
      b.classList.toggle("active", i === j);
    });
    if (icp.traceStatus) $("icp-status").textContent = icp.traceStatus;
    drawIcp();
    drawIcpGrid(); // 清掉格子选中高亮
  }

  // 地形图格子：把上半部动画的数据源切到该格的 iterations
  function selectGridCell(gi) {
    var cell = (icp.gridData.cells || [])[gi];
    if (!cell || !cell.iterations || !cell.iterations.length) return;
    icp.data = icp.gridData; // grid JSON 自带同口径 target/source
    icp.run = { converged: !!cell.converged, iterations: cell.iterations, init_deg: cell.deg };
    icp.gridSel = gi;
    icp.it = 0;
    icp.playing = false;
    $("icp-play").textContent = "▶ 播放迭代";
    var slider = $("icp-slider");
    slider.max = String(cell.iterations.length - 1);
    slider.value = "0";
    document.querySelectorAll("#icp-buttons .init-btn").forEach(function (b) {
      b.classList.remove("active");
    });
    $("icp-status").textContent = "正在回放地形图格子：初值偏差 deg=" + cell.deg + "°、dx=" +
      cell.dx + " m（真值：旋转 " + icp.gridData.truth.deg + "°、平移 [" +
      icp.gridData.truth.t.map(function (v) { return Number(v).toFixed(4); }).join(", ") + "] m）；" +
      "点上方「初值 ×°」按钮可回到单变量实验";
    drawIcp();
    drawIcpGrid();
  }

  /* ---- 收敛域地形图 ---- */

  function buildGridIndex() {
    var g = icp.gridData;
    var degs = g.grid.degs.slice().sort(function (a, b) { return a - b; });
    var dxs = g.grid.dxs.slice().sort(function (a, b) { return a - b; });
    icp.gridDims = { degs: degs, dxs: dxs };
    // 深浅归一化范围：收敛格与失败格各自按 iters 归一
    var ci = [], fi = [];
    (g.cells || []).forEach(function (c) { (c.converged ? ci : fi).push(c.iters || 0); });
    icp.gridRange = {
      c: ci.length ? [Math.min.apply(null, ci), Math.max.apply(null, ci)] : [0, 1],
      f: fi.length ? [Math.min.apply(null, fi), Math.max.apply(null, fi)] : [0, 1]
    };
  }

  function gridCellColor(cell) {
    var range = cell.converged ? icp.gridRange.c : icp.gridRange.f;
    var span = (range[1] - range[0]) || 1;
    var f = ((cell.iters || 0) - range[0]) / span; // 0=轮次最少
    var alpha = 0.88 - 0.55 * f;                   // 轮次越少颜色越深
    return cell.converged
      ? "rgba(52,211,153," + alpha.toFixed(3) + ")"
      : "rgba(248,113,113," + alpha.toFixed(3) + ")";
  }

  function gridCellAt(mx, my) {
    for (var i = 0; i < icp.gridRects.length; i++) {
      var r = icp.gridRects[i];
      if (mx >= r.x && mx <= r.x + r.w && my >= r.y && my <= r.y + r.h) return r;
    }
    return null;
  }

  function gridEventPos(e) {
    var rect = e.currentTarget.getBoundingClientRect();
    return [e.clientX - rect.left, e.clientY - rect.top];
  }

  function onGridHover(e) {
    if (!icp.gridData) return;
    var p = gridEventPos(e);
    var hit = gridCellAt(p[0], p[1]);
    var gi = hit ? hit.gi : -1;
    var canvas = $("icp-grid");
    if (gi === icp.gridHover) return;
    icp.gridHover = gi;
    canvas.style.cursor = hit ? "pointer" : "crosshair";
    var info = $("icp-grid-info");
    if (hit) {
      var c = hit.cell;
      info.textContent = "deg=" + c.deg + "°, dx=" + c.dx + " m ｜ " +
        (c.converged ? "✔ 收敛" : "✘ 未收敛") + " ｜ 迭代 " + c.iters + " 轮 ｜ 末端误差：平移 " +
        c.final_err.t_cm + " cm、角度 " + c.final_err.deg + "° ｜ final_rmse=" + c.final_rmse;
    } else {
      info.textContent = "";
    }
    drawIcpGrid();
  }

  function onGridPick(e) {
    if (!icp.gridData) return;
    var p = gridEventPos(e);
    var hit = gridCellAt(p[0], p[1]);
    if (hit) selectGridCell(hit.gi);
  }

  function drawIcpGrid() {
    if (!icp.gridData || !icp.gridDims) return;
    var canvas = $("icp-grid");
    var w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    var dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(w * dpr));
    canvas.height = Math.max(1, Math.round(h * dpr));
    var ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    var degs = icp.gridDims.degs, dxs = icp.gridDims.dxs;
    var ny = degs.length, nx = dxs.length;
    var padL = 52, padR = 14, padT = 14, padB = 40;
    var cw = (w - padL - padR) / nx, ch = (h - padT - padB) / ny;
    // 轴刻度：纵轴 deg（底部为最小值），横轴 dx
    ctx.font = "11px Consolas, monospace"; ctx.fillStyle = "#8a93a3";
    degs.forEach(function (dg, r) {
      var y = padT + (ny - 1 - r) * ch;
      ctx.fillText(dg + "°", 10, y + ch / 2 + 4);
    });
    dxs.forEach(function (dx, c) {
      var x = padL + c * cw;
      var label = String(dx);
      ctx.fillText(label, x + cw / 2 - label.length * 3.3, h - padB + 16);
    });
    ctx.fillText("dx 平移偏差 (m) →", padL, h - 8);
    ctx.fillText("deg ↑", 10, padT + 2);
    // 格子
    icp.gridRects = [];
    var cells = icp.gridData.cells || [];
    for (var gi = 0; gi < cells.length; gi++) {
      var cell = cells[gi];
      var col = dxs.indexOf(cell.dx), row = degs.indexOf(cell.deg);
      if (col < 0 || row < 0) continue;
      var rx = padL + col * cw, ry = padT + (ny - 1 - row) * ch;
      ctx.fillStyle = gridCellColor(cell);
      ctx.fillRect(rx + 2, ry + 2, cw - 4, ch - 4);
      icp.gridRects.push({ x: rx + 2, y: ry + 2, w: cw - 4, h: ch - 4, gi: gi, cell: cell });
      if (gi === icp.gridSel) { // 选中格：白框
        ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 2;
        ctx.strokeRect(rx + 2, ry + 2, cw - 4, ch - 4);
      } else if (gi === icp.gridHover) { // 悬停格：蓝框
        ctx.strokeStyle = "#4da3ff"; ctx.lineWidth = 1.5;
        ctx.strokeRect(rx + 2, ry + 2, cw - 4, ch - 4);
      }
      // 格内轮次数
      ctx.fillStyle = "rgba(11,14,19,0.85)";
      ctx.font = "bold 11px Consolas, monospace";
      var t = String(cell.iters);
      ctx.fillText(t, rx + cw / 2 - t.length * 3.3, ry + ch / 2 + 4);
      ctx.font = "11px Consolas, monospace";
    }
  }

  function drawGridPlaceholder() {
    var canvas = $("icp-grid");
    var w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    var dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(w * dpr));
    canvas.height = Math.max(1, Math.round(h * dpr));
    var ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = "#5c6675"; ctx.font = "14px sans-serif"; ctx.textAlign = "center";
    ctx.fillText("收敛域地形图数据尚未生成", w / 2, h / 2);
    ctx.textAlign = "left";
  }

  function icpTick(now) {
    if (!icp.playing || !icp.run) return;
    if (!icp.last) icp.last = now;
    if (now - icp.last >= 350) { // 约 3 轮/秒
      icp.last = now;
      icp.it++;
      if (icp.it >= icp.run.iterations.length) {
        icp.it = icp.run.iterations.length - 1;
        icp.playing = false;
        $("icp-play").textContent = "▶ 播放迭代";
      }
      $("icp-slider").value = String(icp.it);
      drawIcp();
    }
    if (icp.playing) icp.raf = requestAnimationFrame(icpTick);
  }

  function drawIcp() {
    if (!icp.data || !icp.run) return;
    var d = icp.data;
    var iter = icp.run.iterations[icp.it];
    var moved = transformPoints(d.source, iter.R, iter.t);
    var plot = icp.plot;
    plot.resize();
    plot.fit([d.target, d.source]);
    plot.clear();
    plot.grid();
    plot.points(d.target, "#8b94a3", 2.5);   // target 灰
    plot.points(moved, "#f59e0b", 2);        // 变换后的 source
    // 徽标与信息
    var badge = $("icp-badge");
    if (icp.run.converged) {
      badge.innerHTML = '<span class="badge badge-ok">✔ 已收敛</span>';
    } else {
      badge.innerHTML = '<span class="badge badge-bad">✘ 未收敛（局部极小）</span>';
    }
    var ang = Math.atan2(iter.R[1][0], iter.R[0][0]) * 180 / Math.PI;
    $("icp-info").textContent =
      "迭代 " + icp.it + "/" + (icp.run.iterations.length - 1) +
      " ｜ rmse = " + iter.rmse + " m ｜ 当前估计：旋转 " + ang.toFixed(2) + "°，平移 [" +
      iter.t.map(function (v) { return Number(v).toFixed(3); }).join(", ") + "]";
    drawIcpCurve();
  }

  // rmse-迭代 曲线（log 纵轴，兼顾 0.012 与 0.3 两个量级）
  function drawIcpCurve() {
    var canvas = $("icp-curve");
    var w = canvas.clientWidth, h = canvas.clientHeight;
    var dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(w * dpr));
    canvas.height = Math.max(1, Math.round(h * dpr));
    var ctx = icp.curve;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    var its = icp.run.iterations;
    var padL = 64, padR = 16, padT = 30, padB = 36;
    var vals = its.map(function (o) { return Math.max(o.rmse, 1e-6); });
    var lo = Math.log10(Math.min.apply(null, vals.concat([1e-3])));
    var hi = Math.log10(Math.max.apply(null, vals.concat([1e-1])));
    lo = Math.floor(lo); hi = Math.ceil(hi);
    if (hi <= lo) hi = lo + 1;
    function X(i) { return padL + (w - padL - padR) * (its.length <= 1 ? 0.5 : i / (its.length - 1)); }
    function Y(v) { return padT + (h - padT - padB) * (1 - (Math.log10(Math.max(v, 1e-6)) - lo) / (hi - lo)); }
    // 网格与刻度
    ctx.strokeStyle = "#242a34"; ctx.fillStyle = "#8a93a3";
    ctx.font = "11px Consolas, monospace"; ctx.lineWidth = 1;
    for (var e = lo; e <= hi; e++) {
      var y = Y(Math.pow(10, e));
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
      ctx.fillText("1e" + e, 8, y + 4);
    }
    ctx.fillText("迭代轮次 →", padL, h - 8);
    ctx.fillText("rmse (m, log)", padL, 16);
    // 曲线
    ctx.strokeStyle = icp.run.converged ? "#34d399" : "#f87171";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(X(0), Y(vals[0]));
    for (var i = 1; i < its.length; i++) ctx.lineTo(X(i), Y(vals[i]));
    ctx.stroke();
    // 当前迭代标记
    ctx.fillStyle = "#4da3ff";
    ctx.beginPath(); ctx.arc(X(icp.it), Y(vals[icp.it]), 4.5, 0, 6.2832); ctx.fill();
  }

  /* ================= 轨迹对比 ================= */

  var traj = { data: null, seq: null, plot: null };

  function initTraj() {
    traj.plot = makePlot($("traj-canvas"));
    window.addEventListener("resize", drawTraj);
    loadData("kitti_traj.json").then(function (res) {
      if (!res) {
        $("traj-status").textContent = srcNote(null);
        return;
      }
      traj.data = res.data;
      var box = $("traj-buttons");
      box.innerHTML = "";
      Object.keys(res.data.sequences).forEach(function (sid) {
        var b = document.createElement("button");
        b.className = "init-btn";
        b.textContent = "序列 " + sid;
        b.addEventListener("click", function () { selectSeq(sid); });
        box.appendChild(b);
      });
      var first = Object.keys(res.data.sequences)[0];
      $("traj-status").textContent = srcNote(res.source) + "；后续将叠加手写 SLAM 的估计轨迹（W3 之后）";
      selectSeq(first);
    });
  }

  function selectSeq(sid) {
    traj.seq = sid;
    document.querySelectorAll("#traj-buttons .init-btn").forEach(function (b) {
      b.classList.toggle("active", b.textContent.indexOf(sid) >= 0);
    });
    drawTraj();
  }

  function drawTraj() {
    if (!traj.data || !traj.seq) return;
    var s = traj.data.sequences[traj.seq];
    var plot = traj.plot;
    plot.resize();
    plot.fit([s.poses]);
    plot.clear();
    plot.grid();
    plot.polyline(s.poses, "#4da3ff", 2);
    plot.points([s.poses[0]], "#34d399", 5);                    // 起点绿
    plot.points([s.poses[s.poses.length - 1]], "#f87171", 5);   // 终点红
    $("traj-info").textContent =
      "序列 " + traj.seq + "（真值）｜ 里程 " + s.length_m + " m ｜ 位姿点数 " + s.poses_raw +
      "（显示降采样至 " + s.poses.length + "）｜ 绿=起点 红=终点";
  }

  /* ================= 学习笔记 ================= */

  // 笔记公式管线：抽取保护数学 -> marked 渲染 -> 回填 KaTeX。
  // 直接拿 marked 产出的 DOM 交给 auto-render 时，公式里的 _ 会先被 Markdown 当强调处理，
  // 文本节点被撕碎后 auto-render 就匹配不到了，所以必须在 markdown 之前先把公式保护起来。
  var MATH_TOKEN = "\u0000MATH";

  function escapeHtmlLite(s) {
    return String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;")
      .replace(/>/g, "&gt;").replace(/"/g, "&quot;");
  }

  // 从原始 md 抽取公式：$$...$$（可跨行）优先，其次 $...$（段内配对、不跨空行、不与 $$ 冲突）。
  // 公式正文保持原样（反斜杠、下划线、百分号均不转义）；代码围栏内一律不抽取。
  function extractMath(src) {
    var map = [];
    function token(tex, display) {
      map.push({ tex: tex, display: display });
      return MATH_TOKEN + (map.length - 1) + "\u0000";
    }
    function protect(chunk) {
      var s = chunk.replace(/\$\$([\s\S]+?)\$\$/g, function (m, body) {
        return token(body, true);
      });
      // 行内 $...$：在空行分隔的同一段落内配对（允许段内换行，但绝不跨空行），
      // 且不与 $$ 冲突；公式正文不做转义。
      s = s.split(/(\n[ \t]*\n+)/).map(function (seg, i) {
        if (i % 2) return seg; // 空行分隔符原样保留
        return seg.replace(/\$(?!\$)([^$]+?)\$(?!\$)/g, function (m, body) {
          if (/^\s|\s$/.test(body)) return m; // "$5 和 $6" 这类不当公式
          return token(body, false);
        });
      }).join("");
      return s;
    }
    var out = [], buf = [], inFence = false;
    function flush() { if (buf.length) { out.push(protect(buf.join("\n"))); buf = []; } }
    src.split(/\r?\n/).forEach(function (line) {
      if (/^\s*(```|~~~)/.test(line)) {
        flush();
        out.push(line);
        inFence = !inFence;
        return;
      }
      if (inFence) out.push(line); else buf.push(line);
    });
    flush();
    return { text: out.join("\n"), map: map };
  }

  // 单个公式转 KaTeX HTML；失败一律回退为 <code> 原文，绝不抛异常
  function mathToHtml(tex, display) {
    var html = null;
    try {
      if (window.katex && window.katex.renderToString) {
        html = window.katex.renderToString(tex, { displayMode: display, throwOnError: false });
      }
    } catch (e) { html = null; }
    if (!html || html.indexOf("katex-error") >= 0) {
      return '<code class="math-fallback">' +
        escapeHtmlLite(display ? "$$" + tex + "$$" : "$" + tex + "$") + "</code>";
    }
    return html;
  }

  function renderNoteHtml(text) {
    var ex = extractMath(text);
    return window.marked.parse(ex.text).replace(/\u0000MATH(\d+)\u0000/g, function (m, i) {
      var item = ex.map[Number(i)];
      return item ? mathToHtml(item.tex, item.display) : m;
    });
  }

  var NOTES = [
    { file: "数据台账_cartographer_paper_deutsches_museum.md", title: "数据台账 · Deutsches Museum" },
    { file: "扫描统计_cartographer_paper_deutsches_museum.md", title: "扫描统计 · Deutsches Museum" },
    { file: "X3数据画像.md", title: "X3 数据画像" },
    { file: "ICP误差-初值实验.md", title: "ICP 误差-初值实验" },
    { file: "ICP理论推导.md", title: "ICP 理论推导" },
    { file: "标定与时间同步.md", title: "标定与时间同步" },
    { file: "环境自检报告.md", title: "环境自检报告" },
    { file: "assets/data/CONTRACT.md", title: "数据契约 CONTRACT" },
    { file: "前端里程计.md", title: "前端里程计" }
  ];

  function initNotes() {
    var list = $("notes-list");
    list.innerHTML = "";
    NOTES.forEach(function (n, i) {
      var a = document.createElement("a");
      a.href = "#/notes/" + i;
      a.textContent = n.title;
      a.addEventListener("click", function (ev) {
        ev.preventDefault();
        selectNote(i);
      });
      list.appendChild(a);
    });
    selectNote(0);
  }

  function selectNote(i) {
    var links = $("notes-list").querySelectorAll("a");
    links.forEach(function (a, j) { a.classList.toggle("active", i === j); });
    var body = $("notes-body");
    body.innerHTML = '<p class="muted">加载中…</p>';
    var n = NOTES[i];
    fetch(encodeURIComponent(n.file).replace(/%2F/g, "/"))
      .then(function (r) {
        if (!r.ok) throw new Error("404");
        return r.text();
      })
      .then(function (text) {
        if (window.marked && window.marked.parse) {
          body.innerHTML = renderNoteHtml(text); // 公式已在 markdown 前抽取，这里回填 KaTeX
        } else {
          var pre = document.createElement("pre");
          pre.textContent = text; // marked 未加载时降级为纯文本
          body.innerHTML = "";
          body.appendChild(pre);
        }
      })
      .catch(function () {
        body.innerHTML = "<h3>" + n.title + "</h3>" +
          '<p class="muted">📝 撰写中——该笔记尚未发布，请先看看其他笔记。</p>';
      });
  }

  /* ================= 启动 ================= */

  var INIT = { slam: initSlam, icp: initIcp, traj: initTraj, notes: initNotes };
  var REDRAW = {
    slam: redrawSlam,
    icp: function () { drawIcp(); drawIcpGrid(); },
    traj: drawTraj
  };

  route();
})();
