# Global trigger used across the lab to coordinate actions such as start/stop,
# readiness checks, map retrieval, or vehicle-specific LED signaling.
# Listened to by all components; some triggers require a valid source and optionally a target.

uint8 START=0             # Start lab operations (e.g. initiate driving)
uint8 STOP=1              # Emergency stop / halt all vehicles
uint8 READY=2             # Component announces its readiness
uint8 MAP_REQUEST=10      # Request static map from map server
uint8 READY_REQUEST=20    # Request readiness confirmation from all components
uint8 LED_REQUEST=30      # Request LED update or status change (component-specific)

builtin_interfaces/Time stamp   # Timestamp of trigger dispatch
uint8 type                      # Trigger type (one of constants above)
string source_id                # ID of the entity sending the request
                                # e.g. "vehicle_1", "planner_3"; must be set for request types
string target_id                # Optional target entity, e.g. for LED_REQUEST:
                                #   source_id = "ips", target_id = "vehicle_5"
                                # Other triggers may leave target empty
