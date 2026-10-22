import * as THREE from 'three';
import { createViewer } from './viewer.js';

const params = new URLSearchParams(window.location.search);
const testMode = params.get('test') === '1';

const container = document.getElementById('viewport');
const viewer = createViewer(container);

let seq = 1;
let ws = null;
let isOwner = false;
let deadmanHeld = false;
let lastJointPositions = null;
let targetDirty = false;

const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const wsUrl = `${proto}//${window.location.host}/ws`;

function connect() {
  ws = new WebSocket(wsUrl);
  ws.onopen = () => {
    if (testMode) {
      send({ topic: '/cmd_control_request' });
    }
  };
  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    handleServerMessage(msg);
  };
  ws.onclose = () => {
    setTimeout(connect, 500);
  };
}

function send(msg) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  msg.seq = seq++;
  ws.send(JSON.stringify(msg));
}

function releaseDeadman() {
  if (!deadmanHeld) return;
  deadmanHeld = false;
  send({ topic: '/cmd_deadman_heartbeat', held: false });
}

function setDeadman(held) {
  if (!isOwner) return;
  if (held && !deadmanHeld) {
    deadmanHeld = true;
  }
  if (deadmanHeld) {
    send({ topic: '/cmd_deadman_heartbeat', held: true });
  }
}

function sendPose() {
  if (!isOwner || !deadmanHeld) return;
  const p = viewer.getTargetPosition();
  const q = viewer.getTargetQuaternion();
  send({
    topic: '/cmd_pose',
    stamp: Date.now() / 1000,
    position: { x: p.x, y: p.y, z: p.z },
    orientation: { x: q.x, y: q.y, z: q.z, w: q.w },
    frame_id: 'base_link',
  });
}

function handleServerMessage(msg) {
  if (msg.topic === '/joint_states') {
    lastJointPositions = msg.position;
    viewer.setJointPositions(msg.name, msg.position);
    document.getElementById('joint-telemetry').textContent = JSON.stringify(msg.position, null, 2);
  }
  if (msg.topic === '/ee_pose') {
    viewer.setTargetPosition(msg.position.x, msg.position.y, msg.position.z);
    viewer.setTargetQuaternion(
      new THREE.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    );
  }
  if (msg.topic === '/safety_state') {
    document.getElementById('safety-mode').textContent = msg.mode;
  }
  if (msg.topic === '/ik_status') {
    document.getElementById('ik-status').textContent = `${msg.target_status} err=${msg.position_error_m.toFixed(4)}`;
  }
  if (msg.topic === '/cmd_control_ack') {
    isOwner = msg.granted;
    document.getElementById('owner-label').textContent = isOwner ? 'Owner' : 'Observer';
  }
}

document.getElementById('btn-request-control').addEventListener('click', () => {
  send({ topic: '/cmd_control_request' });
});

document.getElementById('btn-record-start').addEventListener('click', () => {
  const name = document.getElementById('record-name').value;
  send({ topic: '/cmd_record', action: 'start', name });
});
document.getElementById('btn-record-stop').addEventListener('click', () => {
  const name = document.getElementById('record-name').value;
  send({ topic: '/cmd_record', action: 'stop', name });
});
document.getElementById('btn-replay').addEventListener('click', () => {
  const name = document.getElementById('record-name').value;
  send({ topic: '/cmd_record', action: 'replay', name });
});

viewer.onTargetMoved = () => {
  targetDirty = true;
  sendPose();
};

const keys = {};
const DEADMAN = 'Shift';
const step = 0.01;

window.addEventListener('keydown', (e) => {
  keys[e.key] = true;
  if (e.key === DEADMAN || e.key === 'Shift') {
    setDeadman(true);
  }
  if (e.code === 'Space') {
    send({ topic: '/cmd_estop', engaged: true });
  }
  if (e.key === 'r' || e.key === 'R') {
    send({ topic: '/cmd_reset' });
  }
  if (!isOwner || !deadmanHeld) return;
  const p = viewer.getTargetPosition();
  if (e.key === 'ArrowUp' || e.key === 'w') p.z += step;
  if (e.key === 'ArrowDown' || e.key === 's') p.z -= step;
  if (e.key === 'ArrowLeft' || e.key === 'a') p.y += step;
  if (e.key === 'ArrowRight' || e.key === 'd') p.y -= step;
  if (e.key === 'q') p.x -= step;
  if (e.key === 'e') p.x += step;
  viewer.setTargetPosition(p.x, p.y, p.z);
  sendPose();
});

window.addEventListener('keyup', (e) => {
  keys[e.key] = false;
  if (e.key === 'Shift') {
    releaseDeadman();
  }
});

window.addEventListener('blur', releaseDeadman);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) releaseDeadman();
});

function pollGamepad() {
  const pads = navigator.getGamepads();
  if (!pads || !pads[0]) return;
  const gp = pads[0];
  const dead = gp.buttons[0]?.pressed;
  if (dead) setDeadman(true);
  else if (deadmanHeld) releaseDeadman();
  if (isOwner && deadmanHeld) {
    const p = viewer.getTargetPosition();
    p.x += (gp.axes[0] || 0) * step * 2;
    p.z += (gp.axes[1] || 0) * -step * 2;
    viewer.setTargetPosition(p.x, p.y, p.z);
    sendPose();
  }
}

setInterval(() => {
  if (deadmanHeld) {
    send({ topic: '/cmd_deadman_heartbeat', held: true });
  }
}, 50);

function animate() {
  requestAnimationFrame(animate);
  viewer.resize();
  viewer.render();
  pollGamepad();
}
animate();

connect();

if (testMode) {
  window.__TEST_HOOKS__ = {
    getLinkCount: () => viewer.countArmLinks(),
    getJointPositions: () => lastJointPositions,
    nudgeTargetZ: (dz) => {
      const p = viewer.getTargetPosition();
      viewer.setTargetPosition(p.x, p.y, p.z + dz);
      setDeadman(true);
      sendPose();
    },
    pressKey: (key) => {
      window.dispatchEvent(new KeyboardEvent('keydown', { key, code: key }));
      window.dispatchEvent(new KeyboardEvent('keyup', { key, code: key }));
    },
    requestControl: () => send({ topic: '/cmd_control_request' }),
  };
}
