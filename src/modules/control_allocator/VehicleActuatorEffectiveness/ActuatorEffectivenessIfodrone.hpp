
#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"

#include <uORB/topics/ifodrone_control.h>

#include <uORB/Subscription.hpp>

#include <vector>
#include <px4_platform_common/module_params.h>

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
		static_assert(MAX_NUM_MATRICES >= 2, "expecting at least 2 matrices");
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
	bool _collective_tilt_updated{true};

	uint32_t _motors{};
	uint32_t _untiltable_motors{};

	int _first_control_surface_idx{0}; ///< applies to matrix 1
	int _first_tilt_idx{0}; ///< applies to matrix 0

	float _last_collective_tilt_control{NAN};


private:

	void updateParams() override;

	float _tilt[4]{0.0f, 0.0f, 0.0f, 0.0f};
	float _side_thrust[4]{0.0f, 0.0f, 0.0f, 0.0f};

	float _main_thrust[2]{0.0f, 0.0f};

	uORB::SubscriptionData<ifodrone_control_s> _ifodrone_control_sub{ORB_ID(ifodrone_control)};

	bool _armed{false};
	uint64_t _armed_time{0};
	float _param_spoolup_time{1.f};

	struct ParamHandles {
		param_t com_spoolup_time;
	};

	ParamHandles _param_handles{};

	float _tilt_angles[TILT_MOTORS]{0.0f};

	static constexpr float _min_tilt_angle{math::radians(-45.0f)};
	static constexpr float _max_tilt_angle{math::radians(45.0f)};
	static constexpr float _tilt_deadzone{math::radians(2.0f)};
};
