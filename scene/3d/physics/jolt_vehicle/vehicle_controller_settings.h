#pragma once

#include "core/io/resource.h"

#include "vehicle_differential_settings.h"
#include "vehicle_engine_settings.h"
#include "vehicle_transmission_settings.h"

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Vehicle/VehicleController.h"
#include "Jolt/Physics/Vehicle/WheeledVehicleController.h"


class VehicleControllerSettings : public Resource
{
	GDCLASS(VehicleControllerSettings, Resource);
protected:
	virtual JPH::VehicleControllerSettings get_settings() const = 0;
};

class WheeledVehicleControllerSettings : public VehicleControllerSettings
{
	GDCLASS(WheeledVehicleControllerSettings, VehicleControllerSettings);

protected:
	JPH::WheeledVehicleControllerSettings settings;

	virtual JPH::VehicleControllerSettings get_settings() const override { return settings; }

public:

	Ref<VehicleEngineSettings> get_engine_settings() const { return engine; }
	void set_engine_settings(const Ref<VehicleEngineSettings> &p_engine_settings)
	{
		if (engine.is_valid()) {
			engine->disconnect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_engine));
		}

		engine = p_engine_settings;

		if (!engine.is_valid()) {
			// Reset to default
			engine = memnew(VehicleEngineSettings);
		}

		engine->connect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_engine));
		_apply_engine();
	}

	void set_differentials(const Array<Ref<VehicleDifferentialSettings>> &p_differentials)
	{
		for (const Ref<VehicleDifferentialSettings>& d : differentials)
		{
			if (d.is_valid())
			{
				d->disconnect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_differentials));
			}
		}

		differentials = p_differentials;

		for (const Ref<VehicleDifferentialSettings>& d : differentials)
		{
			if (d.is_valid())
			{
				d->connect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_differentials));
			}
		}

		_apply_differentials();
	}

	VehicleTransmissionSettings	mTransmission;								///< The properties of the transmission (aka gear box)
	float						mDifferentialLimitedSlipRatio = 1.4f;		///< Ratio max / min average wheel speed of each differential (measured at the clutch). When the ratio is exceeded all torque gets distributed to the differential with the minimal average velocity. This allows implementing a limited slip differential between differentials. Set to FLT_MAX for an open differential. Value should be > 1.

protected:
	static void _bind_methods();

	Ref<VehicleEngineSettings> engine;
	Ref<VehicleTransmissionSettings> transmission;
	Array<Ref<VehicleDifferentialSettings>> differentials;

	void _apply_engine()
	{
		settings.mEngine = engine->settings;
	}

	void _apply_transmission()
	{
		settings.mTransmission = transmission->settings;
	}

	void _apply_differentials()
	{
		JPH::Array<JPH::VehicleDifferentialSettings>& array = settings.mDifferentials;
		array.resize(differentials.size());
		int i = 0;
		for (const Ref<VehicleDifferentialSettings>& d : differentials)
		{
			array[i] = d->settings;
		}
	}

};

