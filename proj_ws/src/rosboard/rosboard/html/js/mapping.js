"use strict";

// Mapping controls on the phone: start / pause scanning, save the map and
// finish mapping, without walking back to the robot's touchscreen.
//
// The AMR Control panel owns the real work (slam_toolbox pause toggle,
// map_saver, stopping the bringup). This bar only sends one of four
// commands over the websocket (MSG_MAPPING, validated again in
// rosboard.py) and shows the panel's state from /amr_mapping_state
// (std_msgs/String JSON, ~1 Hz while Mapping runs). The bar is hidden
// unless the panel reports mode == "mapping".

(function () {
  const STATE_TOPIC = "/amr_mapping_state";
  const STALE_MS = 3500;
  let state = null;
  let lastStateAt = 0;

  function defaultName() {
    const d = new Date();
    const p = (n) => String(n).padStart(2, "0");
    return `map_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_` +
           `${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
  }

  function send(action, name) {
    if (!currentTransport || !currentTransport.isConnected()) return;
    currentTransport.mappingCmd({action: action, name: name || ""});
    if (navigator.vibrate) navigator.vibrate(30);
  }

  $(() => {
    const bar = $(`
      <div class="mapping-bar" hidden>
        <div class="mapping-bar-status"></div>
        <button class="mapping-btn mapping-btn-scan"></button>
        <button class="mapping-btn mapping-btn-save">บันทึก<br>แผนที่</button>
        <button class="mapping-btn mapping-btn-finish">สิ้นสุด</button>
      </div>
    `).appendTo("body");
    const statusEl = bar.find(".mapping-bar-status");
    const scanBtn = bar.find(".mapping-btn-scan");
    const saveBtn = bar.find(".mapping-btn-save");
    const finishBtn = bar.find(".mapping-btn-finish");

    function render() {
      const fresh = state && (Date.now() - lastStateAt) < STALE_MS;
      const mapping = fresh && state.mode === "mapping";
      const shown = !!(state && state.mode === "mapping");
      bar.prop("hidden", !shown);
      $("body").toggleClass("mapping-active", shown);
      if (!state) return;
      const busy = !fresh || !state.ready || state.saving;
      scanBtn.html(state.scanning ? "❚❚<br>หยุดสแกน" : "▶<br>เริ่มสแกน");
      scanBtn.toggleClass("scanning", !!state.scanning);
      scanBtn.prop("disabled", !mapping || busy);
      saveBtn.prop("disabled", !mapping || state.saving);
      finishBtn.prop("disabled", !mapping);
      bar.toggleClass("scanning", !!(mapping && state.scanning));
      bar.toggleClass("offline", !fresh);
      statusEl.text(!fresh ? "ขาดการเชื่อมต่อ" : (state.saving ? "กำลังบันทึก"
                    : (state.scanning ? "● สแกนอยู่" : "หยุดอยู่")));
      statusEl.attr("title", state.msg || "");
    }

    scanBtn.on("click", () => send(state && state.scanning ? "pause" : "start"));
    saveBtn.on("click", () => {
      const name = window.prompt("ตั้งชื่อแผนที่ (A-Z a-z 0-9 _ -)", defaultName());
      if (name === null) return;
      send("save", name);
    });
    finishBtn.on("click", () => {
      if (window.confirm("สิ้นสุด Mapping และปิดระบบ?\nแผนที่ที่ยังไม่ได้บันทึกจะหายไป")) {
        send("finish");
      }
    });

    // subscribe once the PIN gate has passed (index.js/auth.js call
    // onAuthenticated on every successful (re)authentication)
    const origOnAuthenticated = onAuthenticated;
    onAuthenticated = function () {
      origOnAuthenticated();
      unsubscribeExtra(STATE_TOPIC, onState);
      subscribeExtra(STATE_TOPIC, "std_msgs/msg/String", onState);
    };
    // Panel messages (saved / name taken / errors) don't fit the narrow
    // column - show each new one briefly as a toast.
    const toast = $('<div class="mapping-toast" hidden></div>').appendTo("body");
    let lastMsg = null, toastTimer = null;
    function showToast(text) {
      toast.text(text).prop("hidden", false);
      if (toastTimer) clearTimeout(toastTimer);
      toastTimer = setTimeout(() => toast.prop("hidden", true), 3500);
    }
    function onState(msg) {
      try { state = JSON.parse(msg.data); } catch (e) { return; }
      if (lastMsg !== null && state.msg && state.msg !== lastMsg) showToast(state.msg);
      lastMsg = state.msg || "";
      lastStateAt = Date.now();
      render();
    }
    setInterval(render, 1000);
  });
})();
