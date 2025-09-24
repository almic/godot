#pragma once

#include "core/io/resource.h"
#include "core/variant/typed_array.h"

#include "vehicle_differential_settings.h"
#include "vehicle_engine_settings.h"
#include "vehicle_transmission_settings.h"

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Vehicle/VehicleController.h"
#include "Jolt/Physics/Vehicle/WheeledVehicleController.h"


class JoltVehicleSettings;

class VehicleControllerSettings : public Resource
{
	GDCLASS(VehicleControllerSettings, Resource);

protected:
	virtual JPH::VehicleControllerSettings* get_settings() { return nullptr; }

	friend class JoltVehicleSettings;
};

class WheeledVehicleControllerSettings : public VehicleControllerSettings
{
	GDCLASS(WheeledVehicleControllerSettings, VehicleControllerSettings);

protected:
	JPH::WheeledVehicleControllerSettings settings;

	virtual JPH::VehicleControllerSettings* get_settings() override { return &settings; }

public:

	WheeledVehicleControllerSettings()
	{
		// When VehicleConstraint takes this, it uses a Ref which will prematurely delete
		// our settings, so we add a reference here to ensure it won't release it for us
		settings.AddRef();
	}

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

	Ref<VehicleTransmissionSettings> get_transmission_settings() const { return transmission; }
	void set_transmission_settings(const Ref<VehicleTransmissionSettings> &p_transmission_settings)
	{
		if (transmission.is_valid()) {
			transmission->disconnect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_transmission));
		}

		transmission = p_transmission_settings;

		if (!transmission.is_valid()) {
			// Reset to default
			transmission = memnew(VehicleTransmissionSettings);
		}

		transmission->connect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_transmission));
		_apply_transmission();
	}

	TypedArray<VehicleDifferentialSettings> get_differentials() const
	{
		TypedArray<VehicleDifferentialSettings> result;
		for (const auto& d : differentials)
		{
			result.push_back(d);
		}
		return result;
	}
	void set_differentials(const TypedArray<VehicleDifferentialSettings> &p_differentials)
	{
		for (const auto& d : differentials)
		{
			if (d.is_valid())
			{
				d->disconnect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_differentials));
			}
		}

		differentials.clear();

		for (const auto& d : p_differentials)
		{
			Ref<VehicleDifferentialSettings> differential = d;

			if (differential.is_valid())
			{
				differential->connect_changed(callable_mp(this, &WheeledVehicleControllerSettings::_apply_differentials));
			}

			differentials.push_back(differential);
		}

		_apply_differentials();
	}

	bool is_open_differential() const { return settings.mDifferentialLimitedSlipRatio == std::numeric_limits<float>::max(); }
	void set_open_differential(bool p_is_open)
	{
		if (p_is_open)
		{
			settings.mDifferentialLimitedSlipRatio = std::numeric_limits<float>::max();
		}
		else
		{
			settings.mDifferentialLimitedSlipRatio = (float) differential_slip_ratio;
		}

		notify_property_list_changed();
	}

	real_t get_differential_slip_ratio() const
	{
		if (is_open_differential())
		{
			return differential_slip_ratio;
		}
		else
		{
			return (real_t) settings.mDifferentialLimitedSlipRatio;
		}
	}
	void set_differential_slip_ratio(real_t p_differential_slip_ratio)
	{
		if (std::isinf(p_differential_slip_ratio) || p_differential_slip_ratio == std::numeric_limits<real_t>::max())
		{
			set_open_differential(true);
		}
		else
		{
			differential_slip_ratio = p_differential_slip_ratio;
			settings.mDifferentialLimitedSlipRatio = (float) p_differential_slip_ratio;
			notify_property_list_changed();
		}
	}

protected:
	static void _bind_methods();

	void _validate_property(PropertyInfo &p_property) const;

	Ref<VehicleEngineSettings> engine;
	Ref<VehicleTransmissionSettings> transmission;
	LocalVector<Ref<VehicleDifferentialSettings>> differentials;

	real_t differential_slip_ratio;

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
		for (const auto& d : differentials)
		{
			if (d.is_valid())
			{
				array[i++] = d->settings;
			}
			else
			{
				ERR_PRINT("VehicleControllerSettings has an invalid setting!");
				array.resize(array.size() - 1);
			}
		}
	}

};

