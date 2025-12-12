import init, { forward_kinematics, inverse_kinematics } from 'reachy-mini';

// Initialize WASM (serial helpers auto-exposed to window!)
await init();

// Expose kinematics functions to window for blockly-app.js
window.forward_kinematics = forward_kinematics;
window.inverse_kinematics = inverse_kinematics;

console.log('WASM module loaded');
