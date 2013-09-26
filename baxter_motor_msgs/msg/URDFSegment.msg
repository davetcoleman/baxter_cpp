#
#  Contains the data for a single URDF segment.
#
# enumeration of geometry types, used in the Link Visual and Collision below
    uint8   SPHERE = 0
    uint8      BOX = 1
    uint8 CYLINDER = 2
    uint8     MESH = 3
#
# Sequence ID, increments when the content changes.
# Sequence ID is associated with link_name to uniquely identify the message content.
uint32                sequence_id
#
# Joint
string                joint_parent
string                joint_name
string                joint_type
geometry_msgs/Pose    joint_origin
geometry_msgs/Vector3 joint_axis
#
# Link
string                link_name
# Link Visual
uint8                 link_visual_type
geometry_msgs/Pose    link_visual_origin
geometry_msgs/Vector3 link_visual_size
#
# Link Collision
uint8                 link_collision_type
geometry_msgs/Pose    link_collision_origin
geometry_msgs/Vector3 link_collision_size
#
# Link Inertial
geometry_msgs/Pose    link_inertial_origin
float64               link_inertial_mass
Inertia               link_inertia
