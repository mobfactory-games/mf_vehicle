
#include <core/class_db.h>
#include "register_types.h"
#include "vehicle.h"
#include "wheel.h"

void register_mf_vehicle_types(void) {
    ClassDB::register_class<mf_vehicle_body>();
    ClassDB::register_class<mf_vehicle_wheel>();
}

void unregister_mf_vehicle_types(void) {
    // yeah, man
}
