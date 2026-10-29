/* SLAM 学习笔记 演示站前端（无框架，Canvas 2D）
 * 数据契约：assets/data/CONTRACT.md（v1.1）
 * 加载顺序：assets/data/<name>.json → assets/data/__fixture__/<name>.json → 占位提示
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

  function srcNote(source) {
    return source ? "数据来源：" + source : "数据尚未生成（将由 tools/07_export_web_data.py 导出）";
  }

  // 等比例坐标系绘图器：自适应 DPR 与容器宽度
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
    return {
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
        var pad = padFrac == null ? 0.08 : padFrac;
        var dx = (maxX - minX) || 1, dy = (maxY - minY) || 1;
        var w = canvas.clientWidth, h = canvas.clientHeight;
        view.scale = Math.min(w / (dx * (1 + pad * 2)), h / (dy * (1 + pad * 2)));
        view.cx = (minX + maxX) / 2;
        view.cy = (minY + maxY) / 2;
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
      // 米制网格 + 坐标轴（x 前 / y 左）
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
        ctx.strokeStyle = "#e8ebf0"; ctx.lineWidth = 1;
        ctx.font = "11px Consolas, monospace"; ctx.fillStyle = "#98a2b3";
        var gx, gy, s;
        for (gx = x0; gx <= x1; gx += step) {
          s = this.toScreen(gx, 0);
          ctx.beginPath(); ctx.moveTo(s[0], 0); ctx.lineTo(s[0], h); ctx.stroke();
          ctx.fillText(trimNum(gx), s[0] + 3, h - 6);
        }
        for (gy = y0; gy <= y1; gy += step) {
          s = this.toScreen(0, gy);
          ctx.beginPath(); ctx.moveTo(0, s[1]); ctx.lineTo(w, s[1]); ctx.stroke();
          ctx.fillText(trimNum(gy), 4, s[1] - 3);
        }
        // 坐标轴
        var o = this.toScreen(0, 0);
        ctx.strokeStyle = "#98a2b3"; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(0, o[1]); ctx.lineTo(w, o[1]); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(o[0], 0); ctx.lineTo(o[0], h); ctx.stroke();
        ctx.fillStyle = "#667085"; ctx.font = "12px sans-serif";
        ctx.fillText("x (前)", w - 46, o[1] - 6);
        ctx.fillText("y (左)", o[0] + 6, 14);
      },
      points: function (pts, color, radius) {
        ctx.fillStyle = color;
        var r = radius || 2;
        for (var i = 0; i < pts.length; i++) {
          var s = this.toScreen(pts[i][0], pts[i][1]);
          ctx.beginPath(); ctx.arc(s[0], s[1], r, 0, 6.2832); ctx.fill();
        }
      },
      polyline: function (pts, color, widthPx) {
        if (!pts.length) return;
        ctx.strokeStyle = color; ctx.lineWidth = widthPx || 2;
        ctx.beginPath();
        var s = this.toScreen(pts[0][0], pts[0][1]);
        ctx.moveTo(s[0], s[1]);
        for (var i = 1; i < pts.length; i++) {
          s = this.toScreen(pts[i][0], pts[i][1]);
          ctx.lineTo(s[0], s[1]);
        }
        ctx.stroke();
      }
    };
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

  var ROUTES = ["home", "notes", "player", "icp", "traj"];
  var inited = {}; // 每个标签页只初始化一次

  function currentRoute() {
    var h = location.hash.replace(/^#\/?/, "");
    return ROUTES.indexOf(h) >= 0 ? h : "home";
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
    } else if (r === "player" || r === "icp" || r === "traj") {
      // 回到标签页时画布尺寸可能变化，重绘一次
      if (REDRAW[r]) REDRAW[r]();
    }
  }

  window.addEventListener("hashchange", route);
  $("nav-toggle").addEventListener("click", function () {
    var nav = $("nav");
    nav.classList.toggle("open");
    $("nav-toggle").setAttribute("aria-expanded", nav.classList.contains("open") ? "true" : "false");
  });

  /* ================= 首页 ================= */

  var ROADMAP = [
    { w: "W1", title: "把数据跑起来", desc: "rosbag 读取、400 帧扫描导出、数据台账与统计画像", status: "done" },
    { w: "W2", title: "标定与时间同步", desc: "KITTI calib.txt 内外参、多传感器时间戳对齐", status: "doing" },
    { w: "W3", title: "手写前端", desc: "帧间匹配（ICP / PL-ICP），亲手复现收敛域问题", status: "todo" },
    { w: "W4", title: "后端", desc: "位姿图优化：把累计误差摊回整条轨迹", status: "todo" },
    { w: "W5", title: "回环与退化", desc: "回环检测兜底漂移；长走廊等退化场景的对策", status: "todo" },
    { w: "W6", title: "多传感器", desc: "IMU / 轮速计融合，运动先验喂给前端", status: "todo" },
    { w: "W7–W12", title: "工程化", desc: "实时化、测试、可视化、文档与开源发布", status: "todo" }
  ];

  function initHome() {
    var box = $("roadmap");
    box.innerHTML = "";
    ROADMAP.forEach(function (it) {
      var div = document.createElement("div");
      div.className = "week " + (it.status === "done" ? "done" : it.status === "doing" ? "doing" : "");
      var badge = it.status === "done" ? '<span class="badge badge-done">已完成</span>'
        : it.status === "doing" ? '<span class="badge badge-doing">进行中</span>'
        : '<span class="badge badge-todo">待开始</span>';
      div.innerHTML = '<div class="wk">' + it.w + " · " + it.title + "</div>" +
        '<p class="muted" style="margin:6px 0 10px;font-size:13px">' + it.desc + "</p>" + badge;
      box.appendChild(div);
    });
  }

  /* ================= 学习笔记 ================= */

  var NOTES = [
    { file: "数据台账_cartographer_paper_deutsches_museum.md", title: "数据台账 · Deutsches Museum" },
    { file: "扫描统计_cartographer_paper_deutsches_museum.md", title: "扫描统计 · Deutsches Museum" },
    { file: "X3数据画像.md", title: "X3 数据画像" },
    { file: "ICP误差-初值实验.md", title: "ICP 误差-初值实验" },
    { file: "标定与时间同步.md", title: "标定与时间同步" },
    { file: "环境自检报告.md", title: "环境自检报告" },
    { file: "assets/data/CONTRACT.md", title: "数据契约 CONTRACT" }
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
          body.innerHTML = window.marked.parse(text);
        } else {
          var pre = document.createElement("pre");
          pre.textContent = text; // marked 未加载时降级为纯文本
          body.innerHTML = "";
          body.appendChild(pre);
        }
      })
      .catch(function () {
        body.innerHTML = '<h3>' + n.title + "</h3>" +
          '<p class="muted">📝 撰写中——该笔记尚未发布，请先看看其他笔记。</p>';
      });
  }

  /* ================= 扫描播放器 ================= */

  var player = { data: null, idx: 0, playing: false, raf: 0, acc: 0, last: 0, plot: null };

  function initPlayer() {
    player.plot = makePlot($("player-canvas"));
    var speedEl = $("player-speed");
    $("player-play").addEventListener("click", function () {
      player.playing = !player.playing;
      $("player-play").textContent = player.playing ? "⏸ 暂停" : "▶ 播放";
      player.last = 0; player.acc = 0;
      if (player.playing) player.raf = requestAnimationFrame(playerTick);
    });
    $("player-slider").addEventListener("input", function () {
      player.idx = parseInt(this.value, 10) || 0;
      drawPlayerFrame();
    });
    window.addEventListener("resize", drawPlayerFrame);
    loadData("scans_seq.json").then(function (res) {
      if (!res) {
        $("player-status").textContent = srcNote(null);
        return;
      }
      player.data = res.data;
      var slider = $("player-slider");
      slider.max = String(res.data.frames.length - 1);
      slider.value = "0";
      player.idx = 0;
      $("player-status").textContent = srcNote(res.source) +
        "；本数据集共 " + (res.data.meta && res.data.meta.frames_total || res.data.frames.length) + " 帧";
      drawPlayerFrame();
    });
  }

  function playerTick(now) {
    if (!player.playing || !player.data) return;
    var frames = player.data.frames;
    var speed = parseFloat($("player-speed").value) || 1;
    if (!player.last) player.last = now;
    player.acc += (now - player.last) / 1000 * speed;
    player.last = now;
    // 用相邻帧真实时间戳差做播放间隔（契约 v1.1：t 为相对秒，首帧 t=0）
    var cur = frames[player.idx];
    var next = frames[Math.min(player.idx + 1, frames.length - 1)];
    var dt = Math.min(1, Math.max(0.01, (next.t - cur.t) || 0.041));
    if (player.acc >= dt) {
      player.acc = 0;
      player.idx++;
      if (player.idx >= frames.length) {
        player.idx = frames.length - 1;
        player.playing = false;
        $("player-play").textContent = "▶ 播放";
      }
      $("player-slider").value = String(player.idx);
      drawPlayerFrame();
    }
    if (player.playing) player.raf = requestAnimationFrame(playerTick);
  }

  function drawPlayerFrame() {
    if (!player.data) return;
    var frames = player.data.frames;
    var f = frames[player.idx];
    var plot = player.plot;
    plot.resize();
    plot.fit([f.xy]);
    plot.clear();
    plot.grid();
    plot.points(f.xy, "#2563eb", 2);
    // 雷达原点
    plot.points([[0, 0]], "#dc2626", 4);
    $("player-info").textContent =
      "帧 " + f.i + " ｜ t = " + Number(f.t).toFixed(3) + " s ｜ 点数 " + f.xy.length +
      " ｜ " + (player.idx + 1) + "/" + frames.length;
  }

  /* ================= ICP 实验 ================= */

  var icp = { data: null, run: null, it: 0, playing: false, raf: 0, last: 0, plot: null, curve: null };

  function initIcp() {
    icp.plot = makePlot($("icp-canvas"));
    icp.curve = $("icp-curve").getContext("2d");
    $("icp-play").addEventListener("click", function () {
      icp.playing = !icp.playing;
      $("icp-play").textContent = icp.playing ? "⏸ 暂停" : "▶ 播放迭代";
      icp.last = 0;
      if (icp.playing) icp.raf = requestAnimationFrame(icpTick);
    });
    $("icp-slider").addEventListener("input", function () {
      icp.it = parseInt(this.value, 10) || 0;
      drawIcp();
    });
    window.addEventListener("resize", drawIcp);
    loadData("icp_trace.json").then(function (res) {
      if (!res) {
        $("icp-status").textContent = srcNote(null);
        return;
      }
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
      $("icp-status").textContent = srcNote(res.source) +
        "；真值（读自数据文件）：旋转 " + res.data.truth.deg + "°，平移 [" +
        res.data.truth.t.map(function (v) { return Number(v).toFixed(4); }).join(", ") + "] m";
      selectRun(0);
    });
  }

  function selectRun(i) {
    icp.run = icp.data.runs[i];
    icp.it = 0;
    icp.playing = false;
    $("icp-play").textContent = "▶ 播放迭代";
    var slider = $("icp-slider");
    slider.max = String(icp.run.iterations.length - 1);
    slider.value = "0";
    document.querySelectorAll("#icp-buttons .init-btn").forEach(function (b, j) {
      b.classList.toggle("active", i === j);
    });
    drawIcp();
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
    plot.points(d.target, "#98a2b3", 2.5);   // target 灰
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
    ctx.strokeStyle = "#e8ebf0"; ctx.fillStyle = "#667085";
    ctx.font = "11px Consolas, monospace"; ctx.lineWidth = 1;
    for (var e = lo; e <= hi; e++) {
      var y = Y(Math.pow(10, e));
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
      ctx.fillText("1e" + e, 8, y + 4);
    }
    ctx.fillText("迭代轮次 →", padL, h - 8);
    ctx.fillText("rmse (m, log)", padL, 16);
    // 曲线
    ctx.strokeStyle = icp.run.converged ? "#16a34a" : "#dc2626";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(X(0), Y(vals[0]));
    for (var i = 1; i < its.length; i++) ctx.lineTo(X(i), Y(vals[i]));
    ctx.stroke();
    // 当前迭代标记
    ctx.fillStyle = "#2563eb";
    ctx.beginPath(); ctx.arc(X(icp.it), Y(vals[icp.it]), 4.5, 0, 6.2832); ctx.fill();
  }

  /* ================= 轨迹对比 ================= */

  var traj = { data: null, seq: null, plot: null };
  var SEQ_COLORS = { est: "#2563eb" };

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
    plot.polyline(s.poses, SEQ_COLORS.est, 2);
    plot.points([s.poses[0]], "#16a34a", 5);                    // 起点绿
    plot.points([s.poses[s.poses.length - 1]], "#dc2626", 5);   // 终点红
    $("traj-info").textContent =
      "序列 " + traj.seq + "（真值）｜ 里程 " + s.length_m + " m ｜ 位姿点数 " + s.poses_raw +
      "（显示降采样至 " + s.poses.length + "）｜ 绿=起点 红=终点";
  }

  /* ================= 启动 ================= */

  var INIT = { home: initHome, notes: initNotes, player: initPlayer, icp: initIcp, traj: initTraj };
  var REDRAW = { player: drawPlayerFrame, icp: drawIcp, traj: drawTraj };

  route();
})();
