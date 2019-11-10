use std::sync::Arc;
use std::thread;
use std::time::Duration;

fn main() {

    // Connect from host computer
    let mut mavconn = mavlink::connect("udpin:0.0.0.0:14550").unwrap();
    // For cross-compiling and putting onto the Raspberry Pi
    //let mut mavconn = mavlink::connect("udpout:0.0.0.0:9000").unwrap();

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
    vehicle
        .send(
            &mavlink::MavHeader::get_default_header(),
            &set_manual_control(),
        )
        .unwrap();

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
        let payload = mavlink::common::MavMessage::ser(&msgs.last().cloned().unwrap());
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
            match data {
                mavlink::common::MavMessage::ATTITUDE(mavlink::common::ATTITUDE_DATA {
                    time_boot_ms,
                    roll,
                    pitch,
                    yaw,
                    rollspeed,
                    pitchspeed,
                    yawspeed,
                }) => println!("Received an attitude: yaw: {}", yaw * 180.0 / 3.14),
                _ => println!("Did not recognize the received data..."),
            }
            //println!("{:?}",data);
        }
    }
}

/// Create a heartbeat message
pub fn heartbeat_message() -> mavlink::common::MavMessage {
    mavlink::common::MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::common::MavType::MAV_TYPE_QUADROTOR,
        autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode: mavlink::common::MavModeFlag::empty(),
        system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
        mavlink_version: 0x3,
    })
}

/// Create a message requesting the parameters list
pub fn request_parameters() -> mavlink::common::MavMessage {
    mavlink::common::MavMessage::PARAM_REQUEST_LIST(mavlink::common::PARAM_REQUEST_LIST_DATA {
        target_system: 0,
        target_component: 0,
    })
}

/// Create a message enabling data streaming
pub fn request_stream() -> mavlink::common::MavMessage {
    mavlink::common::MavMessage::REQUEST_DATA_STREAM(mavlink::common::REQUEST_DATA_STREAM_DATA {
        target_system: 0,
        target_component: 0,
        req_stream_id: 0,
        req_message_rate: 10,
        start_stop: 1,
    })
}

//
fn set_manual_control() -> mavlink::common::MavMessage {
    mavlink::common::MavMessage::COMMAND_LONG(mavlink::common::COMMAND_LONG_DATA {
        param1: 1.0,
        param2: 1.0,
        param3: 400.0,
        param4: 0.0,
        param5: 1.0,
        param6: 0.0,
        param7: 0.0,
        command: mavlink::common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        target_system: 0,
        target_component: 0,
        confirmation: 0,
    })
}
