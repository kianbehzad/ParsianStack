float32 field_length
float32 field_width
float32 goal_width
float32 goal_depth
float32 boundary_width

# The field lines and arcs.
# For easy addressing.
SSLVisionGeomFieldLine top_line
SSLVisionGeomFieldLine bottom_line
SSLVisionGeomFieldLine left_line
SSLVisionGeomFieldLine right_line
SSLVisionGeomFieldLine half_line
SSLVisionGeomFieldLine center_line
SSLVisionGeomFieldLine left_penalty_line
SSLVisionGeomFieldLine right_penalty_line

SSLVisionGeomFieldArc top_left_penalty_arc
SSLVisionGeomFieldArc bottom_left_penalty_arc
SSLVisionGeomFieldArc top_right_penalty_arc
SSLVisionGeomFieldArc bottom_right_penalty_arc
SSLVisionGeomFieldArc center_circle

# All the field lines and arcs again.
# For easy iterating.
SSLVisionGeomFieldLine[] field_lines
SSLVisionGeomFieldArc[] field_arcs
