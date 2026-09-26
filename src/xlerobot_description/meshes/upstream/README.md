Arm, neck and head meshes from `simulation/xlerobot/` (`xlerobot_urdf.zip`) in the
XLeRobot repo, Apache-2.0 like this repo. Used by `urdf/so101_arm.xacro` and
`urdf/head_pantilt.xacro`, which are converted from that URDF. The cart body mesh,
`../raskogbody.stl`, comes from the same place.

`*_part*.ply.convex.stl` are the convex collision pieces of the jaws.

`head_camera_shell.stl` is the camera shell for the cheaper USB camera (the same model as
the wrist cameras), the compact one of the two shells in `hardware/XLeRobot_0_3_0.3mf`
of the XLeRobot repo. Millimetres, +x is the way the lens looks, +z is up, the lens
hole centre is at (11.5, 0, 9.4).
