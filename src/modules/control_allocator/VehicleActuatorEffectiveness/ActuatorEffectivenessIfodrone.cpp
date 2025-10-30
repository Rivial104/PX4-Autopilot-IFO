#include "ActuatorEffectivenessIfodrone.hpp"

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessIfodrone::ActuatorEffectivenessIfodrone(ModuleParams * parent)
	: ModuleParams(parent), _rotors{this}
{
	updateParams();
}
// Definitions for static const basis vectors declared in the header.
const matrix::Vector3f ActuatorEffectivenessIfodrone::_ex = matrix::Vector3f(1.0f, 0.0f, 0.0f);
const matrix::Vector3f ActuatorEffectivenessIfodrone::_ey = matrix::Vector3f(0.0f, 1.0f, 0.0f);
const matrix::Vector3f ActuatorEffectivenessIfodrone::_ez = matrix::Vector3f(0.0f, 0.0f, 1.0f);

void ActuatorEffectivenessIfodrone::updateParams()
{
	ModuleParams::updateParams();

	param_get(_param_handles.com_spoolup_time, &_param_spoolup_time);
}

bool ActuatorEffectivenessIfodrone::getEffectivenessMatrix(Configuration & configuration, EffectivenessUpdateReason external_update)
{
	// const uint64_t now = hrt_absolute_time();

	bool should_update = external_update != EffectivenessUpdateReason::NO_EXTERNAL_UPDATE;

	if(_ifodrone_control_sub.update()) {
		const ifodrone_control_s &ifodrone_control = _ifodrone_control_sub.get();

		for (int i = 0; i < 4; i++) {
			_tilts[i] = ifodrone_control.tilt_angle[i];
			math::constrain(_tilts[i], _min_tilt_angle, _max_tilt_angle);
		}
		should_update = true;
	}

	if (!should_update) {
		return false; // no update needed
	}

	// Rotate the base 'up' vector (ez) around the specified axis to obtain a tilted thrust axis
	_tilted_axis[0] = matrix::Dcmf(matrix::AxisAnglef(_ex, _tilts[0])) * _ez; // forward tilt: rotate up around X
	_tilted_axis[1] = matrix::Dcmf(matrix::AxisAnglef(_ey, _tilts[1])) * _ez; // right tilt: rotate up around Y
	_tilted_axis[2] = matrix::Dcmf(matrix::AxisAnglef(-_ex, _tilts[2])) * _ez; // backward tilt: rotate up around -X
	_tilted_axis[3] = matrix::Dcmf(matrix::AxisAnglef(-_ey, _tilts[3])) * _ez; // left tilt: rotate up around -Y

	// Motors:
	// 0 - Main up
	// 1 - Main down
	// 2 - forward tilt
	// 3- right tilt
	// 4 - backward tilt
	// 5 - left tilt

	// Motors:
	auto &effectiveness_matrix = configuration.effectiveness_matrices[configuration.selected_matrix];

	const unsigned motor_count = MAIN_MOTORS + TILT_MOTORS;

	for(unsigned j=0;j<motor_count;j++)
	{
		matrix::Vector3f thrust_axis;
		// float thrust_scale{1.0f};

		if(j < MAIN_MOTORS) {
			thrust_axis = _ez;
			// thrust_scale = _main_thrust[j];
		} else {
			unsigned side_idx = j - MAIN_MOTORS;
			thrust_axis = _tilted_axis[side_idx];
			// thrust_scale = _side_thrust[side_idx];
		}

		matrix::Vector3f force = _ct * thrust_axis;
		matrix::Vector3f moment = _ct * _motor_positions[j].cross(thrust_axis) - _cm * thrust_axis;

		effectiveness_matrix.slice<3, 1>(0, j) = moment;
		effectiveness_matrix.slice<3, 1>(3, j) = force;
	}
	configuration.actuatorsAdded(ActuatorType::MOTORS, motor_count);


	// Allocation of servos:
	const unsigned servo_start_idx = motor_count;
	const unsigned servo_count = TILT_MOTORS;

	for (unsigned s=0;s<servo_count;++s) {
		unsigned col = servo_start_idx + s; // column index for this actuator in the effectiveness matrix
		matrix::Vector3f axis_of_rotation;

		if (s == 0) { axis_of_rotation = _ex; }      	// front tilt
		else if (s == 1) { axis_of_rotation = _ey; } 	// right tilt
		else if (s == 2) { axis_of_rotation = -_ex; } 	// back tilt
		else { axis_of_rotation = -_ey; } 		// left tilt

		unsigned associated_motor_tilt = MAIN_MOTORS + s;

		matrix::Vector3f d_axis_d_theta = axis_of_rotation.cross(_tilted_axis[s]);

		// float side_thrust_scale = _side_thrust[s];

		effectiveness_matrix.slice<3, 1>(0, col) = _motor_positions[associated_motor_tilt].cross(d_axis_d_theta);
		effectiveness_matrix.slice<3, 1>(3, col) = matrix::Vector3f(0.0f, 0.0f, 0.0f); // Servo produces no direct thrust
	}

	configuration.actuatorsAdded(ActuatorType::SERVOS, servo_count);

	return true;
}

void ActuatorEffectivenessIfodrone::updateSetpoint(const matrix::Vector<float,NUM_AXES>& control_sp, int matrix_index, ActuatorVector & actuator_sp, const ActuatorVector & actuator_min, const ActuatorVector & actuator_max)
{
	const uint64_t now = hrt_absolute_time();

	const float Mx = control_sp(0);
	const float My = control_sp(1);
	const float Mz = control_sp(2);
	const float Fx = control_sp(3);
	const float Fy = control_sp(4);
	const float Fz = control_sp(5);

	const unsigned motor_count = MAIN_MOTORS + TILT_MOTORS; // 6
    	const unsigned servo_start = motor_count;               // 6
    	const unsigned servo_count = TILT_MOTORS;		// 4

	if ((unsigned)actuator_sp.size() < motor_count + servo_count) {
		PX4_WARN("actuator_sp size too small");
		return;
	}

	// MAIN ENGINES //

	float main0 = _ct * Fz;
	float main1 = _ct * Fz;

	float u2 =  _k_side * (Fx * 0.5f) + _k_torque * (My * 0.25f) + 0.0f * Mz;
	float u4 =  _k_side * ( - Fx * 0.5f) + _k_torque * (My * 0.25f) + 0.0f * Mz;
	float u3 =  _k_side * ( + Fy * 0.5f) + _k_torque * (Mx * 0.25f) + 0.0f * Mz;
	float u5 =  _k_side * ( -Fy * 0.5f) + _k_torque * (Mx * 0.25f) + 0.0f * Mz;

	matrix::Vector<float,6> u_mot;
	u_mot(0) = main0;
	u_mot(1) = main1;
	u_mot(2) = u2;
	u_mot(3) = u3;
	u_mot(4) = u4;
	u_mot(5) = u5;

	// Saturation of each control variable
	for (unsigned i = 0; i < motor_count; ++i) {
		// float lb = actuator_min(i);
		// float ub = actuator_max(i);

		// if (!PX4_ISFINITE(lb)) { lb = 0.0f; }
		// if (!PX4_ISFINITE(ub)) { ub = 100.0f; }

		float val = u_mot(i);
		// val = math::constrain(val, lb, ub);
		actuator_sp(i) = val;
	}


	// SERVOS //
	const float tmin = _min_tilt_angle;
	const float tmax = _max_tilt_angle;

	for (unsigned s = 0; s < servo_count; ++s) {
		unsigned idx = servo_start + s;
		float ang = _tilts[s];
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

	if (now - _debug_timer > _debug_timeout_ms * 1000) { // konwersja ms → µs
		_debug_timer = now; // reset timera

	// // --------------- DEBUG ---------------
	// PX4_INFO("Actuator SP: main(%.2f, %.2f) tilts(%.2f, %.2f, %.2f, %.2f) servos(%.2f, %.2f, %.2f, %.2f) \n",
	// 	(double)actuator_sp(0), (double)actuator_sp(1),
	// 	(double)actuator_sp(2), (double)actuator_sp(3), (double)actuator_sp(4), (double)actuator_sp(5),
	// 	(double)actuator_sp(6), (double)actuator_sp(7), (double)actuator_sp(8), (double)actuator_sp(9));
	// }

		// === DEBUG OUTPUT ===
	PX4_INFO("IFO CTRL in: Mx:%.2f My:%.2f Mz:%.2f Fx:%.2f Fy:%.2f Fz:%.2f",
		(double)control_sp(0), (double)control_sp(1), (double)control_sp(2),    // momenty
		(double)control_sp(3), (double)control_sp(4), (double)control_sp(5));

	PX4_INFO("out MOT:[%.2f %.2f %.2f %.2f %.2f %.2f]",
		(double)actuator_sp(0), (double)actuator_sp(1), (double)actuator_sp(2),
		(double)actuator_sp(3), (double)actuator_sp(4), (double)actuator_sp(5));

	PX4_INFO("out SRV:[%.2f %.2f %.2f %.2f]",
		(double)actuator_sp(6), (double)actuator_sp(7), (double)actuator_sp(8), (double)actuator_sp(9));

	}
}



