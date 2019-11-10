#![allow(dead_code)]
#![allow(unused_parens)]

#[derive(Clone, Copy)]
pub struct Attitude {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub rollspeed: f32,
    pub pitchspeed: f32,
    pub yawspeed: f32,
}

impl Attitude {
    pub fn new(
        roll: f32,
        pitch: f32,
        yaw: f32,
        rollspeed: f32,
        pitchspeed: f32,
        yawspeed: f32,
    ) -> Self {
        Attitude {
            roll: roll,
            pitch: pitch,
            yaw: yaw,
            rollspeed: rollspeed,
            pitchspeed: pitchspeed,
            yawspeed: yawspeed,
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
pub fn set_manual_control() -> mavlink::common::MavMessage {
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

pub fn manual_control(
    x: f32,
    y: f32,
    z: f32,
    r: f32,
    buttons: u16,
    target: u8,
) -> mavlink::common::MavMessage {
    mavlink::common::MavMessage::MANUAL_CONTROL(mavlink::common::MANUAL_CONTROL_DATA {
        x: (x * 100.0) as i16, // x, with range from [-1000,1000]
        y: (y * 100.0) as i16, // y, with range from [-1000,1000]
        z: (z * 100.0) as i16, // z, with range from [0,1000]
        r: (r * 100.0) as i16, // rotation r, with range from cw<-[-1000,1000]->ccw
        buttons: buttons,
        target: target,
    })
}
