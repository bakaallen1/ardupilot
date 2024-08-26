// #include "Copter.h"
// #include "mode.h"

// #if MODE_DRAWSQURE_ENABLED == ENABLED

// // Mode initialization
// bool ModeDrawSqure::init(bool ignore_checks)
// {
//     // Initialize stabilize controller, use stabilized collective pitch range
//     copter.input_manager.set_use_stab_col(true);
//     return true;
// }

// // Main run method for the custom mode
// void ModeDrawSqure::run()
// {
//     // Read input from channel 7 (assuming channel 7 is configured for this purpose)
//     int16_t ch7_input = hal.rcin->read(6); // 通道7的索引是6，因为通道索引从0开始
//     motors->rc_write(0, ch7_input);

//         if (!motors->armed()) {
//         // Motors should be Stopped
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
//     } else {
//         // heli will not let the spool state progress to THROTTLE_UNLIMITED until motor interlock is enabled
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
//     }

//     switch (motors->get_spool_state()) {
//     case AP_Motors::SpoolState::SHUT_DOWN:
//         // Motors Stopped
//         attitude_control->reset_yaw_target_and_rate(false);
//         attitude_control->reset_rate_controller_I_terms();
//         break;
//     case AP_Motors::SpoolState::GROUND_IDLE:
//         // If aircraft is landed, set target heading to current and reset the integrator
//         // Otherwise motors could be at ground idle for practice autorotation
//         if ((motors->init_targets_on_arming() && motors->using_leaky_integrator()) || (copter.ap.land_complete && !motors->using_leaky_integrator())) {
//             attitude_control->reset_yaw_target_and_rate(false);
//             attitude_control->reset_rate_controller_I_terms_smoothly();
//         }
//         break;
//     case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
//         if (copter.ap.land_complete && !motors->using_leaky_integrator()) {
//             attitude_control->reset_rate_controller_I_terms_smoothly();
//         }
//         break;
//     case AP_Motors::SpoolState::SPOOLING_UP:
//     case AP_Motors::SpoolState::SPOOLING_DOWN:
//         // do nothing
//         break;
//     }
//     // // Output the PWM value directly to a specific motor, e.g., motor 1
//     // AP_MotorsHeli_Quad* heli_quad_motors = dynamic_cast<AP_MotorsHeli_Quad*>(motors);
//     // heli_quad_motors->rc_write(1, ch7_input);
//     // gcs().send_text(MAV_SEVERITY_INFO, "Motor 1 set to PWM: %d", ch7_input);

// }


// #endif // MODE_DRAWSQURE_ENABLED == ENABLED
