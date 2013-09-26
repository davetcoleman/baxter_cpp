time timestamp

# If an impact is detected, then each entry is the name of an impacted joint
# (e.g., "right_w0")
# Otherwise, this vector is empty.
string[] impact_joints

# If a squish is detected, each entry corresponds to a Cartesian dimension
# that's being squished.  Otherwise, this vector is empty.
int32[] squish_dimensions
