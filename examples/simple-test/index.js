import init, { fk, connect, torque_off, torque_on, record, replay, stop, forward_kinematics, inverse_kinematics } from 'reachy-mini';

// Initialize WASM (serial helpers auto-exposed to window!)
await init();

// Expose WASM functions to window
window.connect = connect;
window.enableTorque = torque_on;
window.disableTorque = torque_off;
window.read_pose = fk;
window.replay = replay;
window.record = record;
window.stop = stop;
window.forward_kinematics = forward_kinematics;
window.inverse_kinematics = inverse_kinematics;

// Enable connect toggle once WASM is loaded
const toggleConnect = document.getElementById('toggle-connect');
if (toggleConnect) {
  toggleConnect.disabled = false;
}

console.log('WASM module loaded');
