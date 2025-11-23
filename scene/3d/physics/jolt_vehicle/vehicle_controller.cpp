#include "vehicle_controller.h"

void WheeledVehicleController::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_driver_input", "forward", "right", "brake", "handbrake"), &WheeledVehicleController::set_driver_input);
	ClassDB::bind_method(D_METHOD("get_current_gear"), &WheeledVehicleController::get_current_gear);
	ClassDB::bind_method(D_METHOD("get_current_rpm"), &WheeledVehicleController::get_current_rpm);
	ClassDB::bind_method(D_METHOD("get_wheel_info"), &WheeledVehicleController::get_wheel_info);
	ClassDB::bind_method(D_METHOD("is_any_driven_wheel_slipping"), &WheeledVehicleController::is_any_driven_wheel_slipping);
}
