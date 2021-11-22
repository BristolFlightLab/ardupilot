#include "Plane.h"

void Plane::update_ml_controller(void) {
	g2.mlController.send_state();
	g2.mlController.update_throttle(SpdHgt_Controller->get_throttle_demand());
	}

#include "MLController.h"
#include <SRV_Channel/SRV_Channel.h>

// Parameter info
const struct AP_Param::GroupInfo MLController::var_info[] = {
	
	// @Param: MLCTRL_OUT2DEG
    // @DisplayName: Output conversion factor
    // @Description: Set the conversion between AP scaled output and physical degrees.
    // @Range: 0 inf
    // @Increment: 0.1
    // @User: User
	AP_GROUPINFO("OUT2DEG", 1, MLController, output2deg, 0.01),
	
	// @Param: MLCTRL_DEG2OUT
    // @DisplayName: Inverse output conversion factor
    // @Description: Set the conversion between physical degrees and AP scaled output. Should be reciprocal of OUT2DEG.
	// @Range: 0 inf
    // @Increment: 0.1
    // @User: User
	AP_GROUPINFO("DEG2OUT", 2, MLController, deg2output, 100.0),
	
	// @Param: MLCTRL_THRAVGT
    // @DisplayName: Time to average throttle over
    // @Description: Set time to average throttle over when using the average throttle mode. Set to zero to use standard mode
	// @Range: 0 10
    // @Increment: 0.1
    // @User: User
	// @Unit: s
	AP_GROUPINFO("THRAVGT", 3, MLController, throttleAverageTime, 0.0),
	
	AP_GROUPEND
	};

MLController::MLController()
	: elev_rate(0.0),
	  elevatorAngle(0.0),
	  sweep_rate(0.0),
	  sweepAngle(0.0),
	  agent_system(-1),
	  agent_component(-1),
	  agent_channel(mavlink_channel_t(-1)),
	  lookupSuccess(false),
	  elevatorAngleUninitialised(true),
	  sweepAngleUninitialised(true),
	  episodeIsComplete(false),
	  in_progress(false),
	  throttleCut(true),
	  throttleFrozen(false),
	  averageThrottle(0),
	  throttleReadings{0},
	  throttleReadingIndex(0)
	{
	}

void MLController::lookup_agent() {
	// Lookup agent in routing table
#ifdef MLDEBUG
	gcs().send_text(MAV_SEVERITY_INFO, "Doing lookup...");
#endif
	lookupSuccess = gcs().chan(0)->find_by_mavtype(MAV_TYPE_ONBOARD_CONTROLLER, agent_system, agent_component, agent_channel);
	}

void MLController::send_state() {
	static unsigned int i = 0;
	
	// Lookup the agent in the routing table. Caching?
	lookup_agent();
	
	// Setup struct
	mavlink_mlagent_state_t state;
	
	// Fill out structure
	
	Vector3f position;
	bool res = plane.ahrs.get_relative_position_NED_origin(position); // Result is NED (m) relative to EKF origin
	if(!res && (++i % 50) == 0) {
		i = 0;
		gcs().send_text(MAV_SEVERITY_ERROR, "[MLAgent] EKF position unavailable");
		//return;
		}
	state.x = position.x;
	state.y = position.y;
	state.z = position.z;
	
	// Euler angles are in radians
	state.phi = plane.ahrs.roll;
	state.theta = plane.ahrs.pitch;
	state.psi = plane.ahrs.yaw;

	Vector3f velocity;
	plane.ahrs.get_velocity_NED(velocity); // Result is NED (m/s)
	// Convert velocity to body axes
	Matrix3f ned2body = plane.ahrs.get_rotation_body_to_ned().transposed(); // Get inverse DCM
	Vector3f velocity_b = ned2body * velocity; // Velocity in body frame
	state.u = velocity_b.x;
	state.v = velocity_b.y;
	state.w = velocity_b.z;
	
	Vector3f angular_rates = plane.ahrs.get_gyro_latest(); // Vector of rotational rates in rad/s (Assume phi,theta,psi)
	state.p = angular_rates.x;
	state.q = angular_rates.y;
	state.r = angular_rates.z;

	state.sweep = sweepAngleUninitialised ? 0.0 : sweepAngle;
	state.elevator = elevatorAngleUninitialised ? 0.0 : elevatorAngle;
	state.tip = plane.experimental_mode_enabled ? 1.0 : 0.0;

	state.va = plane.airspeed.get_airspeed();
	
	// Set targets to match looked-up agent
	state.target_system = agent_system; // 1 for the aircraft?
	state.target_component = agent_component; // ?? For the ML agent?


	
	// Send message to agent
	if( lookupSuccess ) {
#ifdef MLDEBUG
		gcs().send_text(MAV_SEVERITY_ERROR, "[MLAgent] S: %i A: %i", stateSend_ms, actionRecv_ms);
#endif
		mavlink_msg_mlagent_state_send_struct(agent_channel,&state);
#ifdef MLDEBUG
		stateSend_ms = millis();
		gcs().send_text(MAV_SEVERITY_INFO, "MLAGENT_STATE sent to %i", agent_channel);
#endif
		}
	}


void MLController::reset() {
	elevatorAngleUninitialised = true;
	sweepAngleUninitialised = true;
	episodeIsComplete = false;
	sweep_rate = 0.0;
	elev_rate = 0.0;
	lastControlTime = millis();
	throttleCut = false;
	throttleFrozen = false;
	averageThrottle = 0;
	}

// For elevator:
// angle_type = true; high_out is set to 4500
// Scaled value is constrained in range [-4500,4500]
// +-4500 is at limits of servo range with 0 on trim point
// For a 90deg servo deflection, this gives out2deg = 0.01, so deg2out = 100.0

int16_t MLController::get_elevator_output(float timestep) {
	if( elevatorAngleUninitialised ) {
		// Don't have current elevatorAngle, get it
		uint16_t elevatorOutput;
		if( !SRV_Channels::get_output_pwm(SRV_Channel::k_elevator,elevatorOutput) ) {
			// Couldn't get current angle
			gcs().send_text(MAV_SEVERITY_ERROR, "[MLAgent] Could not initialise elevator angle");
			return 1500; // Return something near trim
			}
		// Convert to angle
		elevatorAngle = (float(elevatorOutput) - 1436) / 18.036;
		elevatorAngleUninitialised = false;
		}
#ifdef MLDEBUG
	gcs().send_text(MAV_SEVERITY_INFO, "[MLAgent] Initial elevator angle: %f", elevatorAngle);
#endif
	// Compute updated angle
	elevatorAngle = constrain_float(elevatorAngle + elev_rate * timestep, -25.0, 25.0);
#ifdef MLDEBUG
	gcs().send_text(MAV_SEVERITY_INFO, "[MLAgent] Angle: %f Return %i", elevatorAngle, int16_t(elevatorAngle*deg2output));
#endif
	// Convert to scaled output
	return 18.036 * elevatorAngle + 1436;
	}

int16_t MLController::get_sweep_output(float timestep) {
	if( sweepAngleUninitialised ) {
		sweepAngle = 0.0;
		sweepAngleUninitialised = false;
		}
#ifdef MLDEBUG
	gcs().send_text(MAV_SEVERITY_INFO, "[MLAgent] Initial sweep angle: %f", sweepAngle);
#endif
	// Compute updated angle
	sweepAngle = constrain_float(sweepAngle + sweep_rate * timestep, -10.0, 30.0);
#ifdef MLDEBUG
	gcs().send_text(MAV_SEVERITY_INFO, "[MLAgent] Angle: %f Return %i", sweepAngle, int16_t(sweepAngle*deg2output));
#endif
	// Convert to PWM output
	return (sweepAngle - 94.18137) / -0.06297;
	}

void MLController::handle_message(const mavlink_message_t& message) {
	mavlink_mlagent_action_t action_msg;
	mavlink_msg_mlagent_action_decode(&message, &action_msg);

	if( isnan(action_msg.elevator) || isnan(action_msg.sweep) ) {
		episodeIsComplete = true;
		gcs().send_text(MAV_SEVERITY_INFO, "[MLAgent] Script reports complete");
		return;
		}

	if( action_msg.throttle == 0 ) {
		throttleCut = true;
		}

	// Update rate based on message from agent
	elev_rate = action_msg.elevator;
	sweep_rate = action_msg.sweep;
#ifdef MLDEBUG
	actionRecv_ms = millis();
#endif
	}

bool MLController::is_complete() const {
	return episodeIsComplete;
	}

bool MLController::throttle_is_cut() const {
	return throttleCut;
	}

void MLController::throttle_freeze() {
	throttleFrozen = true;
	if( throttleAverageTime < 0.1 ) {
		// Throttle averaging disabled, skip computation
		return;
	}
	// Calculate the average throttle value
	int nReadings = throttleAverageTime / 0.1;
	int32_t sum = 0.0;
	for( int i = nReadings; i > 0; i--) {
		uint8_t index = (MAX_READINGS + throttleReadingIndex - i) % MAX_READINGS;
		sum += throttleReadings[index];
		}
	averageThrottle = sum / nReadings;
	gcs().send_text(MAV_SEVERITY_INFO, "[MLAgent] Average throttle at freeze = %i ", averageThrottle);
	}

void MLController::update_throttle(float throttle) {
	if(!throttleFrozen) {
		throttleReadings[throttleReadingIndex] = throttle;
		throttleReadingIndex = (throttleReadingIndex + 1) % MAX_READINGS;
		}
	}

int32_t MLController::get_experiment_throttle() {
	if( throttleAverageTime < 0.1 ) {
		return -1;
		}
	else {
		return averageThrottle;
		}
	}
