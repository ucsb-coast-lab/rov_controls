use std::sync::Arc;
use std::thread;
use std::time::Duration;

mod lib;
use crate::lib::*;
use crate::lib::Attitude;

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
    vehicle.send_default(&set_manual_control()).unwrap();
    println!("Protocol version: {:?}", vehicle.get_protocol_version());

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
    let mut attitude: Attitude = Attitude::new(0.0,0.0,0.0,0.0,0.0,0.0);
    loop {
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

        let id = mavlink::common::MavMessage::message_id(&msgs.last().cloned().unwrap());
        //let payload = mavlink::common::MavMessage::ser(&msgs.last().cloned().unwrap());
        //println!("{:?}",payload);
        //println!("{:?}",id);
        let last_msg = &msgs.last().cloned().unwrap();

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
                }) => Attitude::new(roll, pitch, yaw * 180.0 / 3.14, rollspeed, pitchspeed, yawspeed),
                _ => break,
            };
            println!("{:?}", attitude.yaw);
        }

        // Controls section!

        let desired_yaw: f32 = 85.0;
        let diff: f32 = (desired_yaw - attitude.yaw).abs();
        let mut strength = 0.0;
        if diff < 3.0 {
            vehicle.send_default(&manual_control(7, 0, 6, 0, 0, 0)).unwrap();
            println!("yaw: {}, diff: {} < 3.0 => forward motion ",attitude.yaw,diff);
        }
        else {
            if diff > 60.0 {
                strength = 2.2;
            }
            else if diff > 15.0 {
                strength = 1.8;
            }
            else {
                strength = 1.6;
            }
        }

        println!("yaw: {}, diff: {}, strength: {}",attitude.yaw,diff,strength);
        if attitude.yaw < desired_yaw {
            vehicle.send_default(&manual_control(7, 0, 6, (strength as i16), 0, 0)).unwrap();
        }
        else {
            vehicle.send_default(&manual_control(7, 0, 6, (-strength as i16), 0, 0)).unwrap();
        }

        // vehicle.send_default(&manual_control(7, 0, 6, 0, 0, 0)).unwrap();



    }
}
