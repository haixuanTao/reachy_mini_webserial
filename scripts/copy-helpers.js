const fs = require('fs');
const path = require('path');

// Wait for pkg directory to exist (wasm-pack creates it)
const pkgDir = path.join(__dirname, '../pkg');
const helpersSource = path.join(__dirname, '../js/index.js');
const helpersDest = path.join(pkgDir, 'helpers.js');

if (!fs.existsSync(pkgDir)) {
  console.error('pkg/ directory not found. Run wasm-pack build first.');
  process.exit(1);
}

// Read the helpers file
let helpersContent = fs.readFileSync(helpersSource, 'utf8');

// Transform it to export the helper functions instead of importing from pkg
helpersContent = `// Serial Port Helper Functions for Reachy Mini
// This file is auto-generated from js/index.js

// ============ Serial Port Helper ============

let cachedPort = null;

export async function requestSerialPort(forceNew = false) {
  if (cachedPort && !forceNew) {
    console.log('Using cached serial port');
    return cachedPort;
  }

  console.log('Using native WebSerial');
  if (!('serial' in navigator)) {
    throw new Error('WebSerial not available on this browser. Please use Chrome, Edge, or Opera.');
  }

  const port = await navigator.serial.requestPort();
  await port.open({ baudRate: 1000000 });
  cachedPort = port;
  return port;
}

export async function closeSerialPort() {
  if (cachedPort) {
    try {
      await cachedPort.close();
    } catch (e) {
      console.warn('Error closing port:', e);
    }
    cachedPort = null;
  }
}
`;

// Write to pkg/helpers.js
fs.writeFileSync(helpersDest, helpersContent);
console.log('✓ Copied serial helpers to pkg/helpers.js');

// Update pkg/package.json to include helpers.js
const pkgJsonPath = path.join(pkgDir, 'package.json');
const pkgJson = JSON.parse(fs.readFileSync(pkgJsonPath, 'utf8'));

if (!pkgJson.files.includes('helpers.js')) {
  pkgJson.files.push('helpers.js');
  fs.writeFileSync(pkgJsonPath, JSON.stringify(pkgJson, null, 2));
  console.log('✓ Added helpers.js to pkg/package.json files array');
}

// Auto-inject helper setup into pkg/index.js
const pkgIndexPath = path.join(pkgDir, 'index.js');
let pkgIndexContent = fs.readFileSync(pkgIndexPath, 'utf8');

// Check if already patched
if (!pkgIndexContent.includes('AUTO-INJECTED HELPERS')) {
  // Append helper setup code at the end
  const setupCode = `
// AUTO-INJECTED HELPERS - Do not edit manually
import { requestSerialPort as _requestSerialPort, closeSerialPort as _closeSerialPort } from './helpers.js';

// Auto-expose helpers to window for WASM to use
if (typeof window !== 'undefined') {
  window.requestSerialPort = _requestSerialPort;
  window.closeSerialPort = _closeSerialPort;
}
`;

  pkgIndexContent += setupCode;
  fs.writeFileSync(pkgIndexPath, pkgIndexContent);
  console.log('✓ Auto-injected helpers into pkg/index.js');
}
