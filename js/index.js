import { fk, joint_only } from "../pkg/index.js";

import("../pkg/index.js").catch(console.error);

const button_fk = document.getElementById('btn-fk');
button_fk.addEventListener('click', async function() {
await fk();
});

const button_joint_only = document.getElementById('btn-joint-only');
button_joint_only.addEventListener('click', async function() {
await joint_only();
});
// Import the serial helper
import './serial.js';