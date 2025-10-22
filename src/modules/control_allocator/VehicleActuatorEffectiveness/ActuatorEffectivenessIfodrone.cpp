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



	if (_ifodrone_control_sub.update()) {
		const ifodrone_control_s &ifodrone_control = _ifodrone_control_sub.get();

		for(auto i = 0; i < TILT_MOTORS; i++)
		{
			_tilt[i] = ifodrone_control.tilt[i];
			_side_thrust[i] = ifodrone_control.side_thrust[i];
			if(i < MAIN_MOTORS) {
				_main_thrust[i] = ifodrone_control.main_thrust[i];
			}
		}

		PX4_INFO("Tilts updated to: %.2f, %.2f, %.2f, %.2frad", (double)ifodrone_control.tilt[0], (double)ifodrone_control.tilt[1], (double)ifodrone_control.tilt[2], (double)ifodrone_control.tilt[3]);
		should_update = true;
		//
	}

	if (!should_update) {
		return false; // no update needed
	}

	float ct = 4.0f; // thrust coefficient, T = ct * omega^2
	float cm = 1.0f; // moment coefficient, M = cm * omega^2
	auto e1 = matrix::Vector3f(1.0f, 0.0f, 0.0f);
	auto e2 = matrix::Vector3f(0.0f, 1.0f, 0.0f);
	auto e3 = matrix::Vector3f(0.0f, 0.0f, 1.0f);

	matrix::Vector3f tilted_axis[TILT_MOTORS];

	// Rotate the base 'up' vector (e3) around the specified axis to obtain a tilted thrust axis
	tilted_axis[0] = matrix::Dcmf(matrix::AxisAnglef(e1, _tilt[0])) * e3; // forward tilt: rotate up around X
	tilted_axis[1] = matrix::Dcmf(matrix::AxisAnglef(e2, _tilt[1])) * e3; // right tilt: rotate up around Y
	tilted_axis[2] = matrix::Dcmf(matrix::AxisAnglef(-e1, _tilt[2])) * e3; // backward tilt: rotate up around -X
	tilted_axis[3] = matrix::Dcmf(matrix::AxisAnglef(-e2, _tilt[3])) * e3; // left tilt: rotate up around -Y

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
		float thrust_scale{1.0f};

		if(j < MAIN_MOTORS) {
			thrust_axis = (j == 0) ? e3 : -e3; // main motors use up_axis
			thrust_scale = _main_thrust[j];
		} else {
			unsigned side_idx = j - MAIN_MOTORS;
			thrust_axis = tilted_axis[side_idx];
			thrust_scale = _side_thrust[side_idx];
		}

		matrix::Vector3f force = ct * thrust_axis * thrust_scale;
		matrix::Vector3f moment = cm * motor_positions[j].cross(force);

		moment += cm * thrust_scale * e3; // add reaction moment

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

		if (s == 0) { axis_of_rotation = e1; }      	// front tilt
		else if (s == 1) { axis_of_rotation = e2; } 	// right tilt
		else if (s == 2) { axis_of_rotation = -e1; } 	// back tilt
		else { axis_of_rotation = -e2; } 		// left tilt

		unsigned associated_motor = MAIN_MOTORS + s;
		matrix::Vector3f current_thrust_axis = (associated_motor<(int)motor_count) ? ((associated_motor < MAIN_MOTORS) ? ((associated_motor == 0) ? e3 : -e3) : tilted_axis[associated_motor - MAIN_MOTORS]) : e3;

		matrix::Vector3f d_axis_d_theta = axis_of_rotation.cross(current_thrust_axis);

		float side_thrust_scale = _side_thrust[s];

		matrix::Vector3f servo_moment = cm * motor_positions[associated_motor].cross(ct * d_axis_d_theta * side_thrust_scale);
		effectiveness_matrix.slice<3, 1>(0, col) = servo_moment;
		effectiveness_matrix.slice<3, 1>(3, col) = matrix::Vector3f(0.0f, 0.0f, 0.0f); // Servo produces no direct thrust

		configuration.actuatorsAdded(ActuatorType::SERVOS, servo_count);
	}

	return true;
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float,NUM_AXES>& control_sp, int matrix_index, ActuatorVector & actuator_sp, const ActuatorVector & actuator_min, const ActuatorVector & actuator_max)
{
	float tilt_min = _min_tilt_angle;
	float tilt_max = _max_tilt_angle;

	// MAIN ENGINES //
	float thrust_sp = -control_sp(THRUST_Z);
	thrust_sp = math::constrain(thrust_sp, actuator_min(0), actuator_max(0));

	// Check if division is necessary
	actuator_sp(0) = thrust_sp/2.0f;
    	actuator_sp(1) = thrust_sp/2.0f;

	// SIDE ENGINES //
	// float roll_sp  = control_sp(ROLL);
	float pitch_sp = control_sp(PITCH);
	float yaw_sp   = control_sp(YAW);

	actuator_sp(2) = thrust_sp + pitch_sp - yaw_sp;  // front-right
	actuator_sp(3) = thrust_sp + pitch_sp + yaw_sp;  // front-left
	actuator_sp(4) = thrust_sp - pitch_sp - yaw_sp;  // rear-right
	actuator_sp(5) = thrust_sp - pitch_sp + yaw_sp;  // rear-left

	// SIDE TILTS //
	float tilt_sp_rad[4] = {
		_tilt[0],
		_tilt[1],
		_tilt[2],
		_tilt[3]
	};
	for (int i = 0; i < 4; i++) {
		float norm_tilt = 2.0f * (tilt_sp_rad[i] - tilt_min) / (tilt_max - tilt_min) - 1.0f;
		norm_tilt = math::constrain(norm_tilt, -1.0f, 1.0f);
		actuator_sp(6 + i) = norm_tilt;
    	}

	// Saturation of each control variable
	for (int i = 0; i < 10; i++) {
		actuator_sp(i) = math::constrain(actuator_sp(i), actuator_min(i), actuator_max(i));
	}

	// --------------- DEBUG ---------------
	PX4_INFO("Actuator SP: main(%.2f, %.2f) tilts(%.2f, %.2f, %.2f, %.2f) servos(%.2f, %.2f, %.2f, %.2f)",
		(double)actuator_sp(0), (double)actuator_sp(1),
		(double)actuator_sp(2), (double)actuator_sp(3), (double)actuator_sp(4), (double)actuator_sp(5),
		(double)actuator_sp(6), (double)actuator_sp(7), (double)actuator_sp(8), (double)actuator_sp(9));

}



