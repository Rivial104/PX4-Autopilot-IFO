
#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"

#include <uORB/topics/ifodrone_control.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/Subscription.hpp>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <vector>

class ActuatorEffectivenessIfodrone : public ModuleParams, public ActuatorEffectiveness
{
	static constexpr uint8_t TILT_MOTORS{4};
	static constexpr uint8_t MAIN_MOTORS{2};

public:

	ActuatorEffectivenessIfodrone(ModuleParams *parent);
	virtual ~ActuatorEffectivenessIfodrone() = default;

	bool getEffectivenessMatrix(Configuration & configuration, EffectivenessUpdateReason external_update) override;

	int numMatrices() const override { return 2; }

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		// static_assert(MAX_NUM_MATRICES >= 2, "expecting at least 2 matrices");
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
		allocation_method_out[1] = AllocationMethod::PSEUDO_INVERSE;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
		normalize[1] = false;
	}

	const char *name() const override { return "IfoDrone"; }

protected:
	ActuatorEffectivenessRotors _rotors;

	float _main_thrust[2]{0.0f, 0.0f};
	float _side_thrust[4]{0.0f, 0.0f, 0.0f, 0.0f};
	float _tilts[4]{0.0f, 0.0f, 0.0f, 0.0f};

    	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};

private:

	void updateParams() override;

	float _tilt_kp_pitch{1.0f};
	float _tilt_kp_roll{1.0f};

	float _param_spoolup_time{1.0f};

	uint64_t _debug_timeout_ms{5000};
	uint64_t _debug_timer{0};

	struct ParamHandles {
		param_t com_spoolup_time;
	};

	ParamHandles _param_handles{};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<ifodrone_control_s> _ifodrone_control_sub{ORB_ID(ifodrone_control)};

	static constexpr float _min_tilt_angle{math::radians(-45.0f)};
	static constexpr float _max_tilt_angle{math::radians(45.0f)};
	static constexpr float _tilt_deadzone{math::radians(1.0f)};
};
