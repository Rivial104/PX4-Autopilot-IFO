#include "ActuatorEffectivenessIfodrone.hpp"

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>


ActuatorEffectivenessIfodrone::ActuatorEffectivenessIfodrone(ModuleParams *parent)
	: ModuleParams(parent)
{
	updateParams();
}

void ActuatorEffectivenessIfodrone::updateParams()
{
	ModuleParams::updateParams();

	param_get(_param_handles.com_spoolup_time, &_param_spoolup_time);
}

bool ActuatorEffectivenessIfodrone::getEffectivenessMatrix(Configuration & configuration, EffectivenessUpdateReason external_update)
{
	bool should_update = external_update != EffectivenessUpdateReason::NO_EXTERNAL_UPDATE;

	vehicle_attitude_setpoint_s att_sp;
	bool have_att_sp{false};

	if (_att_sp_sub.update(&att_sp)) {
		have_att_sp = true;
	}

	// Convert quaternion to Euler angles
	matrix::Quatf q(att_sp.q_d);
	matrix::Eulerf euler(q);

	float des_pitch{0.0f}, des_roll{0.0f};

	if(have_att_sp) {
		des_pitch = euler.phi();
		des_roll = euler.theta();
	}

	float k_pitch = _tilt_kp_pitch;;
	float k_roll = _tilt_kp_roll;

	float theta_des[4];
	theta_des[0] = k_pitch * des_pitch; // front tilt
	theta_des[1] = k_roll * des_roll;   // right tilt
	theta_des[2] = k_pitch * des_pitch; // back tilt
	theta_des[3] = k_roll * des_roll;   // left tilt

	// Saturate desired tilt angles
	for (int i=0; i<4; i++) {
		if (theta_des[i] > _max_tilt_angle) theta_des[i] = _max_tilt_angle;
		if (theta_des[i] < _min_tilt_angle) theta_des[i] = _min_tilt_angle;
	}

	// apply deadzone and update internal _tilt[] only if changed sufficiently
	bool tilt_changed = false;
	for (int i=0; i<4; i++) {
		if (fabsf(theta_des[i] - _tilt[i]) > _tilt_deadzone) {
			_tilt[i] = theta_des[i];
			tilt_changed = true;
		}
	}

	if (tilt_changed) {
		should_update = true;
	}

	// configuration.selected_matrix = 0;
	// if (_ifodrone_control_sub.update()) {
	// 	const ifodrone_control_s &ifodrone_control = _ifodrone_control_sub.get();

	// 	for(auto i = 0; i < TILT_MOTORS; i++)
	// 	{
	// 		_tilt[i] = ifodrone_control.tilt[i];
	// 		_side_thrust[i] = ifodrone_control.side_thrust[i];
	// 		if(i < MAIN_MOTORS) {
	// 			_main_thrust[i] = ifodrone_control.main_thrust[i];
	// 		}
	// 	}

	// 	PX4_INFO("Tilts updated to: %.2f, %.2f, %.2f, %.2frad", (double)ifodrone_control.tilt[0], (double)ifodrone_control.tilt[1], (double)ifodrone_control.tilt[2], (double)ifodrone_control.tilt[3]);
	// 	should_update = true;
	// 	//
	// }

	if (!should_update) {
		return false; // no update needed
	}

	float ct = 4.0f; // thrust coefficient, T = ct * omega^2
	float cm = 1.0f; // moment coefficient, M = cm * omega^2
	auto ex = matrix::Vector3f(1.0f, 0.0f, 0.0f);
	auto ey = matrix::Vector3f(0.0f, 1.0f, 0.0f);
	auto ez = matrix::Vector3f(0.0f, 0.0f, -1.0f);

	matrix::Vector3f tilted_axis[TILT_MOTORS];

	// Rotate the base 'up' vector (ez) around the specified axis to obtain a tilted thrust axis
	tilted_axis[0] = matrix::Dcmf(matrix::AxisAnglef(ex, _tilt[0])) * ez; // forward tilt: rotate up around X
	tilted_axis[1] = matrix::Dcmf(matrix::AxisAnglef(ey, _tilt[1])) * ez; // right tilt: rotate up around Y
	tilted_axis[2] = matrix::Dcmf(matrix::AxisAnglef(-ex, _tilt[2])) * ez; // backward tilt: rotate up around -X
	tilted_axis[3] = matrix::Dcmf(matrix::AxisAnglef(-ey, _tilt[3])) * ez; // left tilt: rotate up around -Y

	// Motors:
	// 0 - Main up
	// 1 - Main down
	// 2 - forward tilt
	// 3- right tilt
	// 4 - backward tilt
	// 5 - left tilt

	matrix::Vector3f motor_positions[TILT_MOTORS + MAIN_MOTORS] = {
		matrix::Vector3f( 0.0f, 0.0f, 0.1f),
		matrix::Vector3f( 0.0f,  0.0f, -0.1f),

		matrix::Vector3f(0.15f,  0.0f, 0.0f),
		matrix::Vector3f(0.0f,  0.15f, 0.0f),
		matrix::Vector3f(-0.15f,  0.0f, 0.0f),
		matrix::Vector3f(0.0f, -0.15f, 0.0f)
	};

	// Motors:
	auto &effectiveness_matrix = configuration.effectiveness_matrices[configuration.selected_matrix];

	effectiveness_matrix.zero();

	for(int j=0;j<MAIN_MOTORS + TILT_MOTORS;j++)
	{
		matrix::Vector3f thrust_axis;
		// float thrust_scale{1.0f};

		if(j < MAIN_MOTORS) {
			thrust_axis = (j == 0) ? ez : -ez; // main motors use up_axis
			// thrust_scale = _main_thrust[j];
		} else {
			unsigned side_idx = j - MAIN_MOTORS;
			thrust_axis = tilted_axis[side_idx];
			// thrust_scale = _side_thrust[side_idx];
		}

		matrix::Vector3f force = ct * thrust_axis;
		matrix::Vector3f moment = cm * motor_positions[j].cross(force);

		moment += cm * ez; // add reaction moment

		effectiveness_matrix.slice<3, 1>(0, j) = moment;
		effectiveness_matrix.slice<3, 1>(3, j) = force;
	}

	const unsigned motor_count = MAIN_MOTORS + TILT_MOTORS;
	configuration.actuatorsAdded(ActuatorType::MOTORS, motor_count);


	// Allocation of servos:
	const unsigned servo_start_idx = motor_count;
	const unsigned servo_count = TILT_MOTORS;

	for (unsigned s=0;s<servo_count;++s) {
		unsigned col = servo_start_idx + s; // column index for this actuator in the effectiveness matrix
		matrix::Vector3f axis_of_rotation;

		if (s == 0) { axis_of_rotation = ex; }      	// front tilt
		else if (s == 1) { axis_of_rotation = ey; } 	// right tilt
		else if (s == 2) { axis_of_rotation = -ex; } 	// back tilt
		else { axis_of_rotation = -ey; } 		// left tilt

		unsigned associated_motor = MAIN_MOTORS + s;
		matrix::Vector3f current_thrust_axis = (associated_motor<(int)motor_count) ? ((associated_motor < MAIN_MOTORS) ? ((associated_motor == 0) ? ez : -ez) : tilted_axis[associated_motor - MAIN_MOTORS]) : ez;

		matrix::Vector3f d_axis_d_theta = axis_of_rotation.cross(current_thrust_axis);

		float side_thrust_scale = _side_thrust[s];

		matrix::Vector3f servo_moment = cm * motor_positions[associated_motor].cross(ct * d_axis_d_theta * side_thrust_scale);
		effectiveness_matrix.slice<3, 1>(0, col) = servo_moment;
		effectiveness_matrix.slice<3, 1>(3, col) = matrix::Vector3f(0.0f, 0.0f, 0.0f); // Servo produces no direct thrust
	}

	// Register servos once (do not call inside the loop)
	configuration.actuatorsAdded(ActuatorType::SERVOS, servo_count);

	return true;
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float,NUM_AXES>& control_sp, int matrix_index, ActuatorVector & actuator_sp, const ActuatorVector & actuator_min, const ActuatorVector & actuator_max)
{
	const float Fx = control_sp(4);
	const float Fy = control_sp(5);
	const float Fz = control_sp(6);
	const float Mx = control_sp(1);
	const float My = control_sp(2);
	const float Mz = control_sp(3);

	const unsigned motor_count = MAIN_MOTORS + TILT_MOTORS; // 6
    	const unsigned servo_start = motor_count;               // 6
    	const unsigned servo_count = TILT_MOTORS;		// 4

	if ((unsigned)actuator_sp.size() < motor_count + servo_count) {
		PX4_WARN("actuator_sp size too small");
		return;
	}

	// MAIN ENGINES //
	float main0 = 0.5f * Fz;
	float main1 = 0.5f * Fz;

	const float k_side = 5.0f;
	const float k_torque = 0.25f;

	float u2 =  k_side * (Fx * 0.5f) + k_torque * (My * 0.25f) + 0.0f * Mz;
	float u4 =  k_side * ( - Fx * 0.5f) + k_torque * (My * 0.25f) + 0.0f * Mz;
	float u3 =  k_side * ( + Fy * 0.5f) + k_torque * (Mx * 0.25f) + 0.0f * Mz;
	float u5 =  k_side * ( - Fy * 0.5f) + k_torque * (Mx * 0.25f) + 0.0f * Mz;

	matrix::Vector<float,6> u_mot;
	u_mot(0) = main0;
	u_mot(1) = main1;
	u_mot(2) = u2;
	u_mot(3) = u3;
	u_mot(4) = u4;
	u_mot(5) = u5;

	// Saturation of each control variable
	for (unsigned i = 0; i < motor_count; ++i) {
		float lb = actuator_min(i);
		float ub = actuator_max(i);

		if (!PX4_ISFINITE(lb)) { lb = 0.0f; }
		if (!PX4_ISFINITE(ub)) { ub = 10.0f; }

		float val = u_mot(i);
		val = math::constrain(val, lb, ub);
		actuator_sp(i) = val;
	}

	// SERVOS //
	const float tmin = _min_tilt_angle;
	const float tmax = _max_tilt_angle;

	for (unsigned s = 0; s < servo_count; ++s) {
		unsigned idx = servo_start + s;
		float ang = _tilt[s];
		float norm = 2.0f * (ang - tmin) / (tmax - tmin) - 1.0f;
		norm = math::constrain(norm, -1.0f, 1.0f);

		// Respect actuator bounds for servo channels if provided
		float lb = actuator_min(idx);
		float ub = actuator_max(idx);
		if (!PX4_ISFINITE(lb)) lb = -1.f;
		if (!PX4_ISFINITE(ub)) ub = 1.f;
		norm = math::constrain(norm, lb, ub);

		actuator_sp(idx) = norm;
	}

	// --------------- DEBUG ---------------
	PX4_INFO("Actuator SP: main(%.2f, %.2f) tilts(%.2f, %.2f, %.2f, %.2f) servos(%.2f, %.2f, %.2f, %.2f)",
		(double)actuator_sp(0), (double)actuator_sp(1),
		(double)actuator_sp(2), (double)actuator_sp(3), (double)actuator_sp(4), (double)actuator_sp(5),
		(double)actuator_sp(6), (double)actuator_sp(7), (double)actuator_sp(8), (double)actuator_sp(9));

}



