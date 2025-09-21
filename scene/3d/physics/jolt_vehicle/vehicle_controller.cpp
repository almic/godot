#include "vehicle_controller.h"

void WheeledVehicleController::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_driver_input", "forward", "right", "brake", "handbrake"), &WheeledVehicleController::set_driver_input);
}

