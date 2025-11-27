mod dynamixel;
mod kinematics;
use std::cell::RefCell;

use futures_util::StreamExt;
use js_sys::{Boolean, Promise};
use serde::Deserialize;
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::JsFuture;
use web_sys::{console, ReadableStreamDefaultReader, WritableStreamDefaultWriter};

use crate::dynamixel::{
    build_sync_write_position_radians, build_sync_write_torque, parse_status_packet,
    SYNC_READ_POSITION,
};
use crate::kinematics::Kinematics;

thread_local! {
    static PLAYBACK_FRAMES: RefCell<Vec<Vec<f32>>> = RefCell::new(Vec::new());
}

// When the `wee_alloc` feature is enabled, this uses `wee_alloc` as the global
// allocator.
//
// If you don't want to use `wee_alloc`, you can safely delete this.
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

pub async fn sleep(ms: u32) {
    let promise = Promise::new(&mut |resolve, _| {
        web_sys::window()
            .unwrap()
            .set_timeout_with_callback_and_timeout_and_arguments_0(&resolve, ms as i32)
            .unwrap();
    });
    JsFuture::from(promise).await.unwrap();
}

// This is like the `main` function, except for JavaScript.
#[wasm_bindgen(start)]
pub fn main_js() -> Result<(), JsValue> {
    // This provides better error messages in debug mode.
    // It's disabled in release mode so it doesn't bloat up the file size.
    #[cfg(debug_assertions)]
    console_error_panic_hook::set_once();

    // Your code goes here!
    console::log_1(&JsValue::from_str("Hello world!"));

    Ok(())
}

#[wasm_bindgen]
extern "C" {
    fn alert(s: &str);
    // Import the JS function we created
    #[wasm_bindgen(js_namespace = window, catch)]
    async fn requestSerialPort() -> Result<JsValue, JsValue>;

    #[wasm_bindgen(js_name = getCachedPort)]
    fn get_cached_port() -> JsValue;

    #[wasm_bindgen(js_name = closeSerialPort)]
    async fn close_serial_port();

    #[wasm_bindgen(js_name = isOpen)]
    async fn is_open() -> Boolean;
}
/// Generic serial port wrapper that works with both native WebSerial and polyfill
pub struct GenericSerialPort {
    inner: JsValue,
}

impl GenericSerialPort {
    pub fn new(port: JsValue) -> Self {
        Self { inner: port }
    }

    pub async fn open(&self, baud_rate: u32) -> Result<(), JsValue> {
        let options = js_sys::Object::new();
        js_sys::Reflect::set(&options, &"baudRate".into(), &baud_rate.into())?;

        let open_fn = js_sys::Reflect::get(&self.inner, &"open".into())?;
        let open_fn: js_sys::Function = open_fn.dyn_into()?;
        let promise: Promise = open_fn.call1(&self.inner, &options)?.dyn_into()?;
        JsFuture::from(promise).await?;
        Ok(())
    }

    pub fn readable(&self) -> Result<web_sys::ReadableStream, JsValue> {
        let readable = js_sys::Reflect::get(&self.inner, &"readable".into())?;
        readable.dyn_into()
    }

    pub fn writable(&self) -> Result<web_sys::WritableStream, JsValue> {
        let writable = js_sys::Reflect::get(&self.inner, &"writable".into())?;
        writable.dyn_into()
    }
}

#[wasm_bindgen]
pub async fn fk(duration: Option<f64>) {
    let data = String::from_utf8(MOTOR_JSON.to_vec()).unwrap();
    let motors: Vec<Motor> = serde_json::from_str(&data).expect("JSON was not well-formatted");
    let mut kinematics = Kinematics::new(0.038, 0.09);

    for motor in motors {
        let branch_position = nalgebra::Vector3::new(
            motor.branch_position[0],
            motor.branch_position[1],
            motor.branch_position[2],
        );
        let T_motor_world = nalgebra::Matrix4::new(
            motor.T_motor_world[0][0],
            motor.T_motor_world[0][1],
            motor.T_motor_world[0][2],
            motor.T_motor_world[0][3],
            motor.T_motor_world[1][0],
            motor.T_motor_world[1][1],
            motor.T_motor_world[1][2],
            motor.T_motor_world[1][3],
            motor.T_motor_world[2][0],
            motor.T_motor_world[2][1],
            motor.T_motor_world[2][2],
            motor.T_motor_world[2][3],
            motor.T_motor_world[3][0],
            motor.T_motor_world[3][1],
            motor.T_motor_world[3][2],
            motor.T_motor_world[3][3],
        );
        let solution = if motor.solution != 0.0 { 1.0 } else { -1.0 };
        kinematics.add_branch(
            branch_position,
            T_motor_world.try_inverse().unwrap(),
            solution,
        );
    }

    let wait_time = 10;
    // Test inverse kinematics
    let window = web_sys::window().expect("no global `window` exists");
    let document = window.document().expect("should have a document on window");

    let port_js = get_cached_port();

    let port = GenericSerialPort::new(port_js);

    // Now you can read/write
    // Get the readable stream
    let readable = port.readable().unwrap();
    let writable = port.writable().unwrap();
    let reader: ReadableStreamDefaultReader = readable.get_reader().dyn_into().unwrap();
    let writer: WritableStreamDefaultWriter = writable.get_writer().unwrap().dyn_into().unwrap();
    // Read data in a loop
    let motors = vec![1, 2, 3, 4, 5, 6, 21, 22]
        .iter()
        .map(|i| document.get_element_by_id(&format!("motor-{i}")).unwrap())
        .collect::<Vec<_>>();
    let pose_x = document.get_element_by_id("pose-x").unwrap();
    let pose_y = document.get_element_by_id("pose-y").unwrap();
    let pose_z = document.get_element_by_id("pose-z").unwrap();
    let pose_roll = document.get_element_by_id("pose-roll").unwrap();
    let pose_pitch = document.get_element_by_id("pose-pitch").unwrap();
    let pose_yaw = document.get_element_by_id("pose-yaw").unwrap();
    let btn_torque_on = document.get_element_by_id("btn-torque-on").unwrap();
    let btn_torque_off = document.get_element_by_id("btn-torque-off").unwrap();
    let btn_record = document.get_element_by_id("btn-record").unwrap();
    let btn_replay = document.get_element_by_id("btn-replay").unwrap();
    btn_torque_off.set_attribute("disabled", "true").unwrap();
    btn_torque_on.set_attribute("disabled", "true").unwrap();
    btn_record.set_attribute("disabled", "true").unwrap();
    btn_replay.set_attribute("disabled", "true").unwrap();
    let mut results = vec![0.0; 8];
    let start_time = js_sys::Date::now();
    loop {
        // sleep
        JsFuture::from(
            writer.write_with_chunk(&js_sys::Uint8Array::from(&SYNC_READ_POSITION[..]).into()),
        )
        .await
        .unwrap();
        sleep(wait_time).await;
        let result = JsFuture::from(reader.read()).await;
        match result {
            Err(err) => {
                web_sys::console::log_1(
                    &format!("Error reading from serial port: {:?}", err).into(),
                );
            }
            Ok(res) => {
                // Manufacture the element we're gonna append
                let done = js_sys::Reflect::get(&res, &"done".into())
                    .unwrap()
                    .as_bool()
                    .unwrap_or(true);
                let value = js_sys::Reflect::get(&res, &"value".into()).unwrap();
                let data = js_sys::Uint8Array::from(value);
                let bytes = data.to_vec();
                // Parse motor positions
                for motor_id in 0..8 {
                    if let Some((id, pos)) = parse_status_packet(&bytes, motor_id * 15) {
                        let id = if id == 21 {
                            6
                        } else if id == 22 {
                            7
                        } else {
                            id as usize - 1
                        };
                        results[id] = raw_to_radians(pos);
                        let val = &motors[id];
                        val.set_text_content(Some(
                            (raw_to_radians(pos) * 360. / (2. * std::f32::consts::PI))
                                .round()
                                .to_string()
                                .as_str(),
                        ));
                    } else {
                        web_sys::console::log_1(
                            &format!("Failed to parse packet for motor {}", motor_id + 1).into(),
                        );
                    }
                    if done {
                        break;
                    }
                }
                if let Some(dur) = duration {
                    let elapsed = js_sys::Date::now() - start_time;
                    let progress = elapsed / dur;
                    web_sys::console::log_1(&format!("recording {}", progress).into());

                    if progress >= 1.0 {
                        web_sys::console::log_1(&format!("Playback finished").into());
                        break;
                    }
                    PLAYBACK_FRAMES.with_borrow_mut(|frames| {
                        frames.push(results.clone());
                    })
                }
                let t = kinematics.forward_kinematics(results.clone(), None);

                // remove head_z_offset
                let x = -t[(0, 3)] * 1000.; // Reverse X axis because don't know
                let y = t[(1, 3)] * 1000.;
                let z = t[(2, 3)] * 1000.; //- HEAD_Z_OFFSET * 1000.;
                pose_x.set_text_content(Some(&format!("{:.2}", x)));
                pose_y.set_text_content(Some(&format!("{:.2}", y)));
                pose_z.set_text_content(Some(&format!("{:.2}", z)));

                let r = t.fixed_view::<3, 3>(0, 0);
                // Euler XYZ: Roll (X), Pitch (Y), Yaw (Z)
                let pitch = (-r[(2, 0)]).asin();

                let (roll, yaw) = if pitch.cos().abs() > 1e-6 {
                    // Normal case
                    (
                        r[(2, 1)].atan2(r[(2, 2)]), // roll
                        r[(1, 0)].atan2(r[(0, 0)]), // yaw
                    )
                } else {
                    // Gimbal lock
                    ((-r[(1, 2)]).atan2(r[(1, 1)]), 0.0)
                };

                // Convert to degrees
                let roll_deg = roll.to_degrees();
                let pitch_deg = pitch.to_degrees();
                let yaw_deg = yaw.to_degrees();

                pose_roll.set_text_content(Some(&format!("{:.2}", roll_deg)));
                pose_pitch.set_text_content(Some(&format!("{:.2}", pitch_deg)));
                pose_yaw.set_text_content(Some(&format!("{:.2}", yaw_deg)));
                sleep(wait_time).await;
            }
        }
    }

    reader.release_lock();
    writer.release_lock();
    btn_torque_off.remove_attribute("disabled").unwrap();
    btn_torque_on.remove_attribute("disabled").unwrap();
    btn_record.remove_attribute("disabled").unwrap();
    btn_replay.remove_attribute("disabled").unwrap();
    web_sys::console::log_1(&format!("recording {}", 0).into());
}

#[wasm_bindgen]
pub async fn torque_off() {
    let port_js = get_cached_port();

    let port = GenericSerialPort::new(port_js);

    // Now you can read/write
    // Get the readable stream
    let torque_off = build_sync_write_torque(&[1, 2, 3, 4, 5, 6, 21, 22], false);
    let writable = port.writable().unwrap();
    let writer: WritableStreamDefaultWriter = writable.get_writer().unwrap().dyn_into().unwrap();

    // sleep
    JsFuture::from(writer.write_with_chunk(&js_sys::Uint8Array::from(&torque_off[..]).into()))
        .await
        .unwrap();
    writer.release_lock();
}

#[wasm_bindgen]
pub async fn replay() {
    torque_on().await;
    let port_js = get_cached_port();

    let port = GenericSerialPort::new(port_js);

    let writable = port.writable().unwrap();
    let writer: WritableStreamDefaultWriter = writable.get_writer().unwrap().dyn_into().unwrap();
    let frames = PLAYBACK_FRAMES.with_borrow(|frames| frames.clone());
    for frame in frames.iter() {
        let packet = build_sync_write_position_radians(&vec![1, 2, 3, 4, 5, 6, 21, 22], frame);
        JsFuture::from(writer.write_with_chunk(&js_sys::Uint8Array::from(&packet[..]).into()))
            .await
            .unwrap();
        sleep(20).await;
    }
    writer.release_lock();
    torque_off().await;
}

#[wasm_bindgen]
pub async fn connect() {
    let port_js = requestSerialPort()
        .await
        .expect("Failed to get serial port");

    let port = GenericSerialPort::new(port_js);

    port.open(1_000_000).await.expect("Failed to open port");
}

#[wasm_bindgen]
pub async fn torque_on() {
    let port_js = get_cached_port();

    let port = GenericSerialPort::new(port_js);

    // Now you can read/write
    // Get the readable stream
    let torque_on = build_sync_write_torque(&[1, 2, 3, 4, 5, 6, 21, 22], true);

    let writable = port.writable().unwrap();

    let writer: WritableStreamDefaultWriter = writable.get_writer().unwrap().dyn_into().unwrap();

    // sleep
    JsFuture::from(writer.write_with_chunk(&js_sys::Uint8Array::from(&torque_on[..]).into()))
        .await
        .unwrap();
    writer.release_lock();
}

#[allow(non_snake_case)]
#[derive(Deserialize)]
struct Motor {
    branch_position: Vec<f32>,
    T_motor_world: Vec<Vec<f32>>,
    solution: f32,
}
static MOTOR_JSON: [u8; include_bytes!("motors.json").len()] = *include_bytes!("motors.json");

/// Convert raw motor position (0-4095) to radians
/// XL330: 0-4095 maps to 0-360 degrees, center (2048) = 0 radians
fn raw_to_radians(raw: i32) -> f32 {
    ((raw - 2048) as f32 / 4096.0) * 2.0 * std::f32::consts::PI
}

// Convert radians to raw
fn radians_to_raw(radians: f32) -> i32 {
    (radians * 4096.0 / (2.0 * std::f32::consts::PI) + 2048.0) as i32
}
