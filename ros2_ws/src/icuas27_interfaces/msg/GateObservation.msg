# One gate seen by the front camera, expressed in the world frame when the
# detector can triangulate it, otherwise in the camera frame.
std_msgs/Header header
uint8 gate_id                      # 0 = not matched to a course gate
float32 confidence                 # 0..1
bool visible
geometry_msgs/Point position       # gate centre
float32 distance                   # range along the optical axis, metres
float32 bearing_yaw                # horizontal angle to the gate, radians
float32 bearing_pitch              # vertical angle to the gate, radians
string source                      # detector name
