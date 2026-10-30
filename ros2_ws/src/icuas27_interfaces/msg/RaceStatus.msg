# Compact race state for viewers that prefer topics over the JSON contract.
std_msgs/Header header
string state                       # IDLE / TAKEOFF / RACE / LANDING / DONE / ABORTED / SAFE_STOP
uint8 gate_index                   # passes completed in the whole run
uint8 total_gates                  # passes required in the whole run
uint8 target_gate_id               # 0 when no gate is targeted
float32 elapsed                    # raw elapsed seconds, -1 before start
float32 adjusted                   # elapsed + penalties, -1 before start
uint8 penalty_count
bool incomplete                    # a missed gate means the run does not rank
