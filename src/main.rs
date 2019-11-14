use std::sync::Arc;
use std::thread;
use std::time::{Duration, SystemTime};

mod lib;
use crate::lib::Attitude;
use crate::lib::*;

// Will only run thrusters when being run on the Raspberry Pi
fn main() {
    // Connect from host computer
    //let mut mavconn = mavlink::connect("udpin:0.0.0.0:14550").unwrap();
    // For cross-compiling and putting onto the Raspberry Pi
    let mut mavconn = mavlink::connect("udpout:0.0.0.0:9000").unwrap();

    // force the protocol version to mavlink V1:
    mavconn.set_protocol_version(mavlink::MavlinkVersion::V1);

    let vehicle = Arc::new(mavconn);
    vehicle
        .send(
            &mavlink::MavHeader::get_default_header(),
            &request_parameters(),
        )
        .unwrap();
    vehicle
        .send(&mavlink::MavHeader::get_default_header(), &request_stream())
        .unwrap();
    vehicle.send_default(&set_to_manual_control()).unwrap();
    println!(
        "MAVLINK Protocol version: {:?}",
        vehicle.get_protocol_version()
    );

    // Creates a separate thread that keeps a heartbeat between the topside and vehicle computer
    thread::spawn({
        let vehicle = vehicle.clone();
        move || loop {
            let res = vehicle.send_default(&heartbeat_message());
            if res.is_ok() {
                thread::sleep(Duration::from_secs(1));
            } else {
                println!("send failed: {:?}", res);
            }
        }
    });

    let mut msgs: Vec<mavlink::common::MavMessage> = Vec::new();
    let mut attitude: Attitude = Attitude::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // PID control elements
    let setpoint: f32 = 85.0;
    // let mut pid_pre_error: f32 = 0.0;
    let mut pid_integral: f32 = 0.0;
    let kp: f32 = 1.2;
    let ki: f32 = 0.3;
    let kd: f32 = 0.0015;
    let mut yaw_strength: f32 = 0.0;
    let mut fwd_strength: f32 = 0.0;
    let emax: f32 = 5.0;

    println!("About to enter loop:");
    // While the program is active:
    loop {
        let now = SystemTime::now();
        // If the message is received correctly, push that into a queue; else, return the error
        match vehicle.recv() {
            Ok((_header, msg)) => {
                msgs.push(msg);
            }
            Err(e) => {
                match e.kind() {
                    std::io::ErrorKind::WouldBlock => {
                        //no messages currently available to receive -- wait a while
                        thread::sleep(Duration::from_secs(1));
                        continue;
                    }
                    _ => {
                        println!("recv error: {:?}", e);
                        break;
                    }
                }
            }
        }

        // Parse the last message in the queue
        let last_msg = &msgs.last().cloned().unwrap();
        let id = mavlink::common::MavMessage::message_id(last_msg);
        let _payload = mavlink::common::MavMessage::ser(last_msg);
        //println!("{:?}",payload);
        //println!("{:?}",id);

        // Check to see if the received message is of type #30 (ATTITUDE)
        // If so, parse it and assign the data is contains to the pre-initialized Attitude variable
        if id == 30 {
            let data = mavlink::common::MavMessage::parse(
                mavlink::MavlinkVersion::V1,
                last_msg.message_id(),
                &last_msg.ser(),
            )
            .unwrap();
            attitude = match data {
                mavlink::common::MavMessage::ATTITUDE(mavlink::common::ATTITUDE_DATA {
                    time_boot_ms: _,
                    roll,
                    pitch,
                    yaw,
                    rollspeed,
                    pitchspeed,
                    yawspeed,
                }) => Attitude::new(
                    roll,
                    pitch,
                    yaw * 180.0 / 3.14, // yaw is now going to be in degrees instead of radians
                    rollspeed,          // However, the "speed" variables should stil be in rad/s
                    pitchspeed,
                    yawspeed * 180.0 / 3.14, //
                ),
                _ => break, // If it's not an Attitude message, then we need to break off
            };
            // println!("{:?}", attitude.yaw);
        }

        // Controls section!
        // Sets the desired yaw, and calculates how far from the desired
        let error: f32 = (setpoint - attitude.yaw).abs();
        let dt = match now.elapsed() {
            Ok(elapsed) => {
                // it prints '2'
                let dt = (elapsed.as_nanos() as f32) / 1_000_000_000.0;
                // println!("dt = {}", dt);
                dt
            }
            Err(e) => {
                // an error occurred!
                println!("Error: {:?}", e);
                0.0
            }
        };

        if dt == 0.0 {
            println!("dt == 0; breaking...");
            break;
        }
        pid_integral = pid_integral + (error * dt);
        yaw_strength = (kp * error.abs()) + (ki * pid_integral) + (kd * attitude.yawspeed / dt);
        // strength = strength + output;

        /*
        if error < 3.0 {
            vehicle
                .send_default(&manual_control(7.0, 0.0, 6.5, 0.0, 0, 0))
                .unwrap();
            println!(
                "yaw: {}, error: {} < 3.0 => forward motion ",
                attitude.yaw, error
            );
        } else {
            if error > 60.0 {
                strength = 2.2;
            } else if error > 15.0 {
                strength = 1.8;
            } else {
                strength = 1.6;
            }
        }
        */

        println!(
            "yaw: {:.2}, yawspeed: {:.2}, error: {:.2}, pid_integral: {:.2}, yaw_strength: {:.2}",
            attitude.yaw, attitude.yawspeed, error, pid_integral, yaw_strength
        );
        if error < emax && attitude.yaw < setpoint {
            vehicle
                .send_default(&manual_control(3.5, 0.0, 6.2, yaw_strength/100.0, 0, 0))
                .unwrap();
        }
        else if error < emax && attitude.yaw > setpoint {
            vehicle
                .send_default(&manual_control(3.5, 0.0, 6.2, -yaw_strength/100.0, 0, 0))
                .unwrap();
        }
        else if attitude.yaw < setpoint {
            vehicle
                .send_default(&manual_control(0.0, 0.0, 6.2, yaw_strength/100.0, 0, 0))
                .unwrap();
        } else {
            vehicle
                .send_default(&manual_control(0.0, 0.0, 6.2, -yaw_strength/100.0, 0, 0))
                .unwrap();
        }

    }
}
