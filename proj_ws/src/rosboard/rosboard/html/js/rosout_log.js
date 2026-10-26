"use strict";

// /rosout log on the main (joystick / mapping) page: is the Jetson keeping
// up while scanning? rosboard.py passes /rosout unthrottled (max 30
// lines/s). "Overload" lines = the usual signs a node can't keep up (SLAM /
// costmaps dropping scans, controller missing its rate, TF too old); their
// count over the last 60 s is shown on the toggle button, so it turns red
// without opening the panel. Same rules as goal_picker's Log panel.

(function () {
  const TOPIC = "/rosout";
  const LOG_MAX = 300;
  const LEVELS = {10: "D", 20: "I", 30: "W", 40: "E", 50: "F"};
  const LOAD_RE = /dropping message|queue is full|Control loop missed|missed its desired rate|extrapolation into the (past|future)|timed out waiting for transform|message filter/i;
  let lines = [];      // {t, lvl, text, load}
  let loadTimes = [];  // ms timestamps of overload lines
  let warnOnly = false;
  let paused = false;
  let dirty = false;

  function onLog(msg) {
    const lvl = LEVELS[msg.level] || "?";
    const load = LOAD_RE.test(msg.msg || "");
    const now = Date.now();
    lines.push({t: now, lvl: lvl, text: "[" + lvl + "] " + (msg.name || "") + ": " + (msg.msg || ""), load: load});
    if (lines.length > LOG_MAX) lines.splice(0, lines.length - LOG_MAX);
    if (load) loadTimes.push(now);
    dirty = true;
  }

  function loadCount() {
    const cut = Date.now() - 60000;
    while (loadTimes.length && loadTimes[0] < cut) loadTimes.shift();
    return loadTimes.length;
  }

  $(() => {
    const btn = $('<button class="rosout-toggle-btn" title="Log (/rosout)">Log</button>').appendTo("body");
    const panel = $(`
      <div class="rosout-panel" hidden>
        <div class="rosout-row">
          <button class="rosout-btn rosout-warn"></button>
          <button class="rosout-btn rosout-pause"></button>
          <button class="rosout-btn rosout-clear">ล้าง</button>
        </div>
        <div class="rosout-sum"></div>
        <div class="rosout-lines"></div>
      </div>
    `).appendTo("body");
    const box = panel.find(".rosout-lines")[0];
    const sum = panel.find(".rosout-sum");

    function renderButtons() {
      panel.find(".rosout-warn").text(warnOnly ? "แสดงทั้งหมด" : "เฉพาะ WARN/ERROR");
      panel.find(".rosout-pause").text(paused ? "เลื่อนต่อ" : "หยุดเลื่อน");
    }

    function update(force) {
      const n = loadCount();
      btn.text(n ? "Log ⚠" + n : "Log");
      btn.toggleClass("bad", n > 0);
      if (panel.prop("hidden")) return;
      sum.toggleClass("bad", n > 0);
      sum.text(n ? "ไม่ไหว: " + n + " ข้อความ overload ใน 60 วิ (แถวที่ไฮไลต์)"
                 : "ไม่มีข้อความ overload ใน 60 วิ");
      if (paused || !(force || dirty)) return;
      dirty = false;
      box.innerHTML = "";
      for (const l of lines) {
        if (warnOnly && !l.load && (l.lvl === "D" || l.lvl === "I")) continue;
        const d = document.createElement("div");
        d.className = "rosout-" + l.lvl + (l.load ? " rosout-load" : "");
        d.textContent = new Date(l.t).toTimeString().slice(0, 8) + " " + l.text;
        box.appendChild(d);
      }
      box.scrollTop = box.scrollHeight;
    }

    btn.on("click", () => {
      const open = panel.prop("hidden");
      panel.prop("hidden", !open);
      btn.toggleClass("active", open);
      if (open) { renderButtons(); update(true); }
    });
    panel.find(".rosout-warn").on("click", () => { warnOnly = !warnOnly; renderButtons(); update(true); });
    panel.find(".rosout-pause").on("click", () => { paused = !paused; renderButtons(); update(true); });
    panel.find(".rosout-clear").on("click", () => { lines = []; loadTimes = []; update(true); });

    // subscribe once the PIN gate has passed (same hook as mapping.js)
    const origOnAuthenticated = onAuthenticated;
    onAuthenticated = function () {
      origOnAuthenticated();
      unsubscribeExtra(TOPIC, onLog);
      subscribeExtra(TOPIC, "rcl_interfaces/msg/Log", onLog);
    };
    setInterval(() => update(false), 500);
  });
})();
