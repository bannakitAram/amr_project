"use strict";

// PIN gate: before any topic data (or the joystick) is usable, the client
// must submit the access PIN shown on the AMR Control app's screen next to
// the QR code. The server (rosboard.py) generates a fresh random PIN each
// time it starts and ignores every other message from a socket until that
// socket authenticates - see handlers.py.
//
// Once a PIN is accepted it's cached in localStorage so reconnects (WiFi
// blips, the 5s auto-reconnect loop in index.js) re-authenticate silently
// instead of nagging the operator every time. A rosboard restart rotates
// the PIN, which invalidates the cached one and brings the gate back.

let _pendingAuthCode = null;

function onAuthGateOpen() {
  const stored = localStorage.getItem('rosboard_pin');
  if (stored) {
    _pendingAuthCode = stored;
    currentTransport.authenticate({code: stored});
  } else {
    showAuthGate();
  }
}

function onAuthResult(result) {
  if (result && result.ok) {
    if (_pendingAuthCode) {
      localStorage.setItem('rosboard_pin', _pendingAuthCode);
    }
    hideAuthGate();
    onAuthenticated();
  } else {
    localStorage.removeItem('rosboard_pin');
    showAuthGate('รหัสไม่ถูกต้อง กรุณาลองใหม่');
  }
}

function showAuthGate(errorText) {
  $('#auth-gate-error').text(errorText || '');
  $('#auth-gate').prop('hidden', false);
  const input = document.getElementById('auth-gate-input');
  input.value = '';
  input.focus();
}

function hideAuthGate() {
  $('#auth-gate').prop('hidden', true);
}

$(() => {
  $('#auth-gate-form').on('submit', (e) => {
    e.preventDefault();
    const code = $('#auth-gate-input').val().trim();
    if (!code) return;
    _pendingAuthCode = code;
    currentTransport.authenticate({code: code});
  });
});
