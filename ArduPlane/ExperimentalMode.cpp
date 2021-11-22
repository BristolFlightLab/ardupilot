#include "Plane.h"

bool Plane::set_experimental_mode(bool enabled) {
	// Called on waypoint entry with enabled=true
	// Called in other places with enabled=false for failsafe + mode switches
	if( !allow_experimental_mode ) {
		experimental_mode_enabled = false;
		return false;
		}
	experimental_mode_enabled = enabled;
	if (enabled == true) {
		// Reset on transition to true
		g2.mlController.reset();
		// Freeze mlController throttle average
		g2.mlController.throttle_freeze();
		}
	return true;
	}
