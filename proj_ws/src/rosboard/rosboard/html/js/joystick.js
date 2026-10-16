"use strict";

// Virtual joystick for driving the AMR from the rosboard mobile web page.
// Hold-to-drive: sends teleop commands only while the knob is actively
// held, and force-sends a zero command the instant it's released (plus a
// server-side deadman watchdog in rosboard.py as defense in depth).
//
// Frontend clamps mirror the backend defaults in rosboard.py - the backend
// re-clamps independently, so a mismatch here is not a safety issue, just
// a feel/tuning one.
const JOYSTICK_MAX_LINEAR = 0.1;   // m/s
const JOYSTICK_MAX_ANGULAR = 0.15; // rad/s - kept low so fast in-place turns during Mapping don't hurt SLAM scan-matching
const JOYSTICK_SEND_INTERVAL_MS = 100; // well under the backend's 500ms deadman timeout

(function () {
  let panelOpen = false;
  let dragging = false;
  let sendTimer = null;
  let knobX = 0, knobY = 0; // -1..1, current commanded axes

  function sendCurrent() {
    if (!currentTransport || !currentTransport.isConnected()) return;
    currentTransport.teleop({
      linear: -knobY * JOYSTICK_MAX_LINEAR,
      angular: -knobX * JOYSTICK_MAX_ANGULAR,
    });
  }

  function sendStop() {
    knobX = 0;
    knobY = 0;
    if (currentTransport && currentTransport.isConnected()) {
      currentTransport.teleop({linear: 0, angular: 0});
    }
  }

  // Confirms to the operator, via vibration + a brief flash, that the
  // deadman has been released and the stop command was actually sent -
  // otherwise a released finger gives no feedback that driving has stopped.
  let stoppedFlashTimer = null;
  function confirmStopped(padEl) {
    if (navigator.vibrate) navigator.vibrate(40);
    if (!padEl) return;
    padEl.classList.add('joystick-pad--stopped');
    if (stoppedFlashTimer) clearTimeout(stoppedFlashTimer);
    stoppedFlashTimer = setTimeout(() => {
      padEl.classList.remove('joystick-pad--stopped');
      stoppedFlashTimer = null;
    }, 250);
  }

  function setKnobFromEvent(padEl, knobEl, clientX, clientY) {
    const rect = padEl.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    const radius = rect.width / 2;

    let dx = (clientX - cx) / radius;
    let dy = (clientY - cy) / radius;
    const mag = Math.sqrt(dx * dx + dy * dy);
    if (mag > 1) {
      dx /= mag;
      dy /= mag;
    }

    knobX = dx;
    knobY = dy;
    knobEl.style.transform = `translate(${dx * radius}px, ${dy * radius}px)`;
  }

  function resetKnob(knobEl) {
    knobEl.style.transform = "translate(0px, 0px)";
  }

  $(() => {
    const toggleBtn = $('<button class="joystick-toggle-btn" title="เปิด/ปิด ควบคุมขับหุ่น">🕹</button>').appendTo('body');
    const panel = $(`
      <div class="joystick-panel" hidden>
        <div class="joystick-stop-btn">หยุด (STOP)</div>
        <div class="joystick-pad">
          <div class="joystick-knob"></div>
        </div>
      </div>
    `).appendTo('body');

    const padEl = panel.find('.joystick-pad')[0];
    const knobEl = panel.find('.joystick-knob')[0];
    const stopBtn = panel.find('.joystick-stop-btn')[0];

    toggleBtn.on('click', () => {
      panelOpen = !panelOpen;
      panel.prop('hidden', !panelOpen);
      toggleBtn.toggleClass('active', panelOpen);
      if (!panelOpen) {
        sendStop();
        resetKnob(knobEl);
      }
    });

    stopBtn.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      dragging = false;
      sendStop();
      resetKnob(knobEl);
      confirmStopped(padEl);
    });

    padEl.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      dragging = true;
      padEl.setPointerCapture(e.pointerId);
      setKnobFromEvent(padEl, knobEl, e.clientX, e.clientY);
      if (!sendTimer) sendTimer = setInterval(sendCurrent, JOYSTICK_SEND_INTERVAL_MS);
    });

    padEl.addEventListener('pointermove', (e) => {
      if (!dragging) return;
      setKnobFromEvent(padEl, knobEl, e.clientX, e.clientY);
    });

    function endDrag(e) {
      if (!dragging) return;
      dragging = false;
      if (sendTimer) {
        clearInterval(sendTimer);
        sendTimer = null;
      }
      sendStop();
      resetKnob(knobEl);
      confirmStopped(padEl);
    }
    padEl.addEventListener('pointerup', endDrag);
    padEl.addEventListener('pointercancel', endDrag);

    // Safety net: if the tab is hidden/backgrounded mid-drive, stop.
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) endDrag();
    });
    window.addEventListener('beforeunload', () => {
      if (dragging) sendStop();
    });
  });
})();
