import { fk, connect, torque_off, torque_on, replay } from "../pkg/index.js";

import("../pkg/index.js").catch(console.error);

const button_fk = document.getElementById('btn-fk');
button_fk.addEventListener('click', async function() {
    fk();
});

const button_torque_on = document.getElementById('btn-torque-on');
button_torque_on.addEventListener('click', async function() {
await torque_on();
});
const button_torque_off = document.getElementById('btn-torque-off');
button_torque_off.addEventListener('click', async function() {
await torque_off();
});
const button_connect = document.getElementById('btn-connect');
button_connect.addEventListener('click', async function() {
await connect();
});
const button_record = document.getElementById('btn-record');
button_record.addEventListener('click', async function() {
await fk(10000);
});

const button_replay = document.getElementById('btn-replay');
button_replay.addEventListener('click', async function() {
await replay();
});

// Import the serial helper
import './serial.js';