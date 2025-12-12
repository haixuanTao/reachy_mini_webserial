# Reachy Mini 🤖

> Browser-based control library for Reachy Mini robot using WebAssembly

[![npm](https://img.shields.io/npm/v/reachy-mini.svg)](https://www.npmjs.com/package/reachy-mini)
[![WebAssembly](https://img.shields.io/badge/WebAssembly-654FF0?logo=webassembly&logoColor=fff)](https://webassembly.org/)

High-performance JavaScript package for controlling the Reachy Mini humanoid robot head. Built with Rust/WASM for real-time kinematics and motor control.

## Features

- 🚀 WebAssembly-powered kinematics (forward & inverse)
- 🔌 WebSerial (USB) or WebSocket connectivity
- 📱 Cross-platform (Desktop Chrome/Edge, Android with WebUSB)
- 🎬 Record and replay robot movements
- 🔧 Direct Dynamixel XL330 motor control (8 servos, IDs 11-18)

## Installation

```bash
npm install reachy-mini
```

## Usage in HTML (CDN)

```html
<!DOCTYPE html>
<html>
<head>
    <title>Reachy Mini Control</title>
</head>
<body>
    <h1>Reachy Mini Control</h1>
    <button id="connect-btn">Connect</button>
    <button id="enable-btn">Enable Torque</button>
    <button id="test-btn">Test Kinematics</button>
    <button id="disable-btn">Disable Torque</button>

    <script type="module">
        // Import from CDN (no npm install needed!)
        import init, {
            connect,
            torque_on,
            torque_off,
            forward_kinematics,
            inverse_kinematics
        } from 'https://unpkg.com/reachy-mini@0.2.0';

        // Initialize WASM module
        await init();

        document.getElementById('connect-btn').onclick = async () => {
            await connect();
            console.log('Connected!');
        };

        document.getElementById('enable-btn').onclick = async () => {
            await torque_on();
            console.log('Motors enabled');
        };

        document.getElementById('test-btn').onclick = () => {
            const pose = forward_kinematics([0, 0, 0, 0, 0, 0, 0, 0]);
            console.log('Pose:', pose);
            const joints = inverse_kinematics([0, 0, 0, 0, 0, 0]);
            console.log('Joints:', joints);
        };

        document.getElementById('disable-btn').onclick = async () => {
            await torque_off();
            console.log('Motors disabled');
        };
    </script>
</body>
</html>
```

**Note:** Must be served via HTTP (not file://) for ES modules to work. Use `python3 -m http.server 8080` or any local server.

## API

### Connection
- `connect()` - Connect via WebSocket (ws://localhost:8000) or WebSerial
- `enableTorque()` - Enable all 8 motors
- `disableTorque()` - Disable all motors (compliant mode)

### Kinematics
- `forward_kinematics(angles)` - Joint angles (deg) → [x, y, z, roll, pitch, yaw] (mm, deg)
- `inverse_kinematics(pose)` - [x, y, z, roll, pitch, yaw] → joint angles (deg)
- `read_pose(duration?)` - Continuously read and update pose

### Recording
- `record()` - Record movement for 10 seconds at 50Hz
- `replay()` - Replay recorded movement
- `stop()` - Stop recording/playback

## Development

```bash
npm install          # Install dependencies
npm start            # Dev server with hot reload
npm run build        # Production build
npm test             # Run tests
```

## Browser Support

Chrome/Edge 89+ (WebSerial), Firefox/Safari (WebSocket only), Android Chrome (WebUSB)

## Hardware

- Reachy Mini robot head with 8× Dynamixel XL330 servos
- USB-to-serial adapter: Arduino, FTDI, CP210x, CH340, Adafruit, or RPi Pico
- Baud rate: 1,000,000

## Examples

```bash
# Standalone HTML example (no npm install needed!)
cd examples/basic
python3 -m http.server 8080
# Open http://localhost:8080/index.html

# Full-featured interface
cd examples/simple-test && npm install && npm start

# Visual programming with Blockly
cd examples/blockly && npm install && npm start
```

## License

MIT - Xavier Tao (tao.xavier@outlook.com)
