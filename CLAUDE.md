# Project notes for Claude

This is the ROS 2 Jazzy stack for XLeRobot 0.4.0, two-wheel base. See
ARCHITECTURE.md for the real design doc, this file is just operating notes.

## Isaac Sim digital twin

Real hardware isn't reachable at all times, so there's an Isaac Sim twin for
testing the stack (Nav2, diff_drive_controller, safety supervisor) without
it. Setup and the full pipeline are in docs/sim-twin.md, read that first.

Quick facts:
- Isaac Sim 6.1.0, pip install in ~/venvs/isaacsim (Python 3.12 venv), not
  the Omniverse-launcher bundle style.
- Sim side of ros2_control uses `joint_state_topic_hardware_interface`
  (ros-controls, official), NOT sts_hardware_interface's mock mode. Launch
  xacro with `use_sim:=true` for this path, see
  xlerobot_description/urdf/xlerobot.ros2_control_sim.xacro.
- The Omniverse MCP server (github.com/serionist/omniverse-mcp) is
  registered in ~/Developer/XLeRobot/.mcp.json. Its AI guide is copied in
  below per its README's setup instructions.
- The sim twin does NOT reproduce R1 (the STS3215 encoder wrap bug), that's
  real-hardware-only. It's for everything else.

---

## Context

You are connected to a running NVIDIA Omniverse application (Isaac Sim 5.1+, USD Composer, etc.) via the Omniverse MCP. The scene is a USD stage -- a hierarchy of "prims" with paths like `/World/Robot/body`. Objects only move during physics playback; scene edits are instant.

**Always call `get_sim_state()` first** to learn the up axis (Y or Z), whether physics is running, and the scene scale.

## Recommended Companion: Context7 MCP

AI models are not trained on Omniverse/Isaac Sim APIs. Before writing any `execute_script` code, use the Context7 MCP to look up the correct API:
- Library ID: `isaac-sim/isaacsim`
- Example: "how to set joint positions on an articulation"

This prevents hallucinated API calls. Without it, `execute_script` code will frequently fail.

## Workflow

```
1. get_sim_state()           -> learn up axis, sim state
2. capture_viewport()        -> see the scene (image + bboxes + segmentation)
3. get_scene_tree()          -> understand the prim hierarchy
4. [make changes]            -> create/transform prims, spawn robots
5. capture_viewport()        -> verify changes visually
6. sim_control("play")       -> start physics
7. start_recording() + play  -> capture behavior over time
8. stop_recording()          -> review frames
```

### Geometry analysis workflow
```
1. get_prim_face_count_tree("/World")  -> see where geometry budget is spent
2. get_mesh_stats("/World/Component")  -> detailed per-mesh breakdown
3. [simplify / create shell]
4. compare_prims(prim_path="/World/Component", variant_set="model", variant_a="focused", variant_b="shell")  -> validate reduction
```

### Variant workflow (shell creation)
```
1. create_variant_structure("/World/Server", "model", variant_names=["focused", "shell"])
2. [populate focused variant with full geometry]
3. [populate shell variant with simplified geometry]
4. set_variant_selection("/World/Server", "model", "shell")  -> switch & refresh viewport
5. compare_prims(prim_path="/World/Server", variant_set="model", variant_a="focused", variant_b="shell")
```

### Asset preparation workflow
```
1. flatten_usd("D:/work/component_flat.usdc", "D:/source/component.usda")  -> resolve references
2. export_prim_as_file("/World/Component", "D:/export/component.usdc")     -> self-contained file
3. update_material_paths("/Old/Looks", "/World/Looks", "/World")           -> fix material paths
```

## Tool Quick Reference

| Tool | Purpose |
|------|---------|
| **Scene** | |
| `get_sim_state()` | Up axis, sim state, prim count, meters_per_unit -- **call first** |
| `get_scene_tree(root, max_depth)` | Prim hierarchy overview (small scenes) |
| `dump_scene(root, max_depth, include_properties, filter_types, property_filter)` | Full scene dump to file (large scenes) |
| `get_prim_properties(prim_path)` | All USD properties on a single prim |
| `get_prim_bounds(prim_path)` | World-space bounding box (center, dims, min, max) |
| `create_prim(prim_path, prim_type, usd_path, position, rotation, scale, enable_physics)` | Create prim or reference a USD asset |
| `delete_prim(prim_path)` | Remove a prim and its descendants |
| `set_prim_transform(prim_path, position, rotation, scale)` | Move / rotate / scale a prim |
| `clone_prim(source_path, target_path, count, offset)` | Duplicate a prim (optionally N copies with offset) |
| `set_visibility(prim_path, visible)` | Show / hide a prim |
| `save_scene(file_path)` | Save stage to disk |
| `new_scene()` | Create a fresh empty stage |
| **Geometry & Mesh** | |
| `get_mesh_stats(prim_path)` | Face / vertex / triangle counts per mesh |
| `get_prim_face_count_tree(root, max_depth)` | Hierarchical face count summary |
| `compare_prims(prim_path_a, prim_path_b)` | Compare two prims (geometry, bounds, materials) |
| `compare_prims(prim_path, variant_set, variant_a, variant_b)` | Compare two variants on the same prim |
| **USD Operations** | |
| `flatten_usd(output_path, input_path)` | Resolve all references into a single flat file |
| `export_prim_as_file(prim_path, output_path)` | Export subtree as self-contained USD |
| **Variants** | |
| `create_variant_structure(prim_path, variant_set_name, variant_names, default_variant)` | Create a variant set with named variants |
| `set_variant_selection(prim_path, variant_set, variant_name)` | Switch active variant (handles FSD/Hydra) |
| **Materials** | |
| `set_material(prim_path, color, opacity, roughness, metallic, material_path)` | Create and bind a PBR material |
| `update_material_paths(old_prefix, new_prefix, prim_path)` | Bulk-fix material binding paths |
| **Camera & Visual** | |
| `set_camera(position, target)` | Position the viewport camera |
| `look_at_prim(prim_path, distance, azimuth, elevation)` | Point camera at a prim |
| `inspect_prim(prim_path, angles, width, height, distance, include_segmentation)` | 14-angle cube-based orbit capture (6 faces + 8 corners) |
| `capture_viewport(width, height, camera_path, include_image)` | Screenshot + bounding boxes + segmentation (set `include_image=True` to see it inline) |
| `viewport_light(action, enabled)` | Check/toggle viewport camera light and list scene lights |
| **Simulation** | |
| `sim_control(action)` | play / pause / stop / step |
| `start_recording(fps, width, height, camera_path, track_prims, property_filter)` | Begin frame capture |
| `stop_recording()` | End recording, write metadata |
| `get_recording_frame(frame_index, session_dir)` | Retrieve a recorded frame |
| **Physics** | |
| `set_physics_properties(prim_path, mass, density, friction, restitution)` | Set physics params |
| `apply_force(prim_path, force, position, impulse)` | Apply force or impulse to a rigid body |
| `raycast(origin, direction, max_distance)` | Cast a physics ray |
| **Robots** | |
| `create_robot(robot_type, prim_path, position, rotation)` | Spawn a robot from the built-in library |
| `get_robot_info(prim_path)` | Joint list, DOF count, link count |
| `get_joint_states(prim_path)` | Current joint positions (sim must have played) |
| `set_joint_targets(prim_path, targets)` | Set joint position targets |
| **Debug & Extensions** | |
| `draw_debug(shape, ...)` | Draw debug lines / spheres / points |
| `manage_extensions(action, extension_id, search, enabled_only)` | List / enable / disable Kit extensions |
| **Logging** | |
| `get_logs(count, min_level, channel, search)` | Recent Omniverse log entries -- call after errors to diagnose |
| **Scripting** | |
| `execute_script(code)` | Run Python inside Omniverse (supports `await mcp.*` for async MCP calls) |

## Best Practices

### Always look before you act

`capture_viewport()` is your primary tool for understanding the scene. It saves three files every time:
1. **Viewport image** (PNG) -- what the camera sees
2. **Bounding boxes** (TXT) -- screen-space pixel coordinates + world-space bounds for every visible prim
3. **Instance segmentation** (PNG + legend TXT) -- each prim rendered as a unique color, with a legend mapping colors to prim paths

**Inline image:** Use `capture_viewport(include_image=True)` to also return the viewport image inline as an `ImageContent` block. This lets you see the scene directly without reading the file. The image uses ~50K tokens of context, so only enable it when you need to visually inspect the scene. Files are always saved regardless.

Segmentation requires the `omni.syntheticdata` extension (enabled by default in Isaac Sim). The MCP Bridge auto-initializes the sensor at startup. If segmentation returns empty, the sensor may need a frame to warm up -- recapture once. If `omni.syntheticdata` is not available (e.g., plain USD Composer), segmentation files will be missing but viewport + bounding boxes still work.

**Use capture_viewport liberally.** Call it at the start to understand the scene, after every significant change to verify results, and whenever you need spatial information.

### Scene discovery

When exploring an unfamiliar scene:
1. `capture_viewport()` -- see the full scene, read the bounding box file to identify what's where
2. `get_scene_tree()` -- understand the prim hierarchy
3. Move the camera with `set_camera()` or `look_at_prim()` to examine specific areas
4. Use `inspect_prim()` to orbit-capture an object from multiple angles
5. Hide objects with `set_visibility(path, false)` then `capture_viewport()` again to see what's behind/inside them
6. Re-show with `set_visibility(path, true)` when done

### Spatial reasoning

Use bounding boxes for accurate placement:
```
1. get_prim_bounds("/World/desk")   -> center, dimensions
2. get_prim_bounds("/World/apple")  -> center, dimensions
3. set_prim_transform("/World/apple", position=[computed])
4. capture_viewport()               -> verify placement visually
```

Cross-reference the bounding box file from `capture_viewport()` with the viewport image -- the screen_bbox tells you exactly where each prim appears in the image, so you can verify positions match your intent.

### Investigating complex scenes

For scenes with many nested or overlapping objects:
- **Peel layers**: Hide outer objects (`set_visibility`) to reveal internal structure, capture, then restore
- **Use segmentation**: The instance segmentation image lets you distinguish overlapping objects that look similar in the viewport -- each gets a unique color
- **Filter by region**: Read the bounding box file and focus on prims whose screen_bbox falls in the area of interest
- **Orbit captures**: Use `inspect_prim()` to see an object from multiple angles when a single view isn't enough
- **World bounds**: The bounding box file includes `world_center` and `world_dimensions` for every visible prim -- use these for spatial math without needing `get_prim_bounds()` calls

### Verification loop

After any change, verify:
```
[make change] -> capture_viewport() -> check image + bboxes -> adjust if needed
```
Do not assume changes worked. Always visually confirm.

## Output Files

- Viewport captures -> `mcp_output/captures/viewport_NNNN.png`
- Bounding boxes -> `mcp_output/captures/viewport_NNNN_bboxes.txt`
- Segmentation -> `mcp_output/captures/viewport_NNNN_segmentation_NNNN.png` + `_legend.txt`
- Large text -> `mcp_output/responses/*.txt`
- Scene dumps -> `mcp_output/scene_dump.txt`
- Recordings -> `mcp_output/recordings/rec_TIMESTAMP/`

Data uses prim-block format (`[/path]` headers, `key = value` lines), not JSON.

## execute_script

Pre-loaded: `omni`, `carb`, `Usd`, `UsdGeom`, `UsdLux`, `UsdPhysics`, `Sdf`, `Gf`, `mcp`. Set `result = <value>` to return data.

**`mcp` bridge:** The `mcp` object provides async access to MCP handlers from within scripts. Use `await` to call methods — async mode is auto-detected:

```python
# Capture, analyze, reposition, recapture — all in one script
viewport = await mcp.capture_viewport(width=1280)
bounds = await mcp.prim_bounds("/World/Robot")
await mcp.set_camera([bounds["result"]["center"][0] + 3, 3, 3])
logs = await mcp.get_logs(count=10, min_level="warn")
result = {"bounds": bounds["result"], "warnings": len(logs["result"]["entries"])}
```

**Fallback strategy:** Every MCP tool can be replicated via `execute_script` using the Omniverse Python API directly. If a tool returns an error, you can fall back to writing the equivalent Python code. Use Context7 (`isaac-sim/isaacsim`) or the [NVIDIA Omniverse docs](https://docs.omniverse.nvidia.com/) to find the correct API.

## Available Robots (Isaac Sim)

franka, ur10, carter, jetbot, g1, go1, go2, h1, spot, anymal

## Common Pitfalls

- **Wrong up axis**: Always check with `get_sim_state()` before placing objects
- **Blind edits**: Always `capture_viewport()` after changes -- don't assume they worked
- **Object falls through ground**: Missing `UsdPhysics.CollisionAPI`
- **Joint states empty**: Sim must have played at least once
- **API guessing**: Use Context7 -- don't guess Omniverse APIs
- **Large scenes**: Use `dump_scene` instead of `get_scene_tree` with properties
- **Tool errors**: If any MCP tool fails, call `get_logs(min_level="warn")` to see Omniverse's internal errors, then fall back to `execute_script` with the equivalent Python code
- **Can't see inside**: Hide outer prims with `set_visibility` to reveal internal structure
- **Variant switching via script**: Never use `execute_script` to switch variants -- viewport will show stale geometry. Always use `set_variant_selection()` which handles FSD detection and Hydra refresh automatically
- **Stale bounding boxes**: After variant switches, bounding box data may be cached. `compare_prims` and `set_variant_selection` handle this automatically by clearing `BBoxCache`
- **Material paths after copy**: When copying USD assets to a new directory, use `update_material_paths()` to fix broken material references
- **MDL compilation delay**: First load of MDL materials shows fallback colors. Wait or recapture -- don't assume materials are broken
- **Layer caching**: `flatten_usd` and `export_prim_as_file` handle `Sdf.Layer` caching automatically. If using `execute_script` for similar operations, use `Sdf.Layer.Find(path).Clear()` before creating new layers
- **Camera positioning in scripts**: Do NOT set camera via `UsdGeom.Xformable` or `ViewportCameraState.set_position_world()` in `execute_script` -- neither reliably moves the viewport camera. Use `await mcp.set_camera()` or `await mcp.look_at()` within the script, or use the MCP tools between script calls
- **Camera capture timing**: After `set_camera()`, wait 20+ frames before capturing. After variant switch with FSD on, wait 30+ frames. `inspect_prim` and `set_variant_selection` handle this automatically
- **Variant comparison captures**: When capturing pairs (e.g., focused vs shell), capture both at the same camera position before moving. Never capture all of one variant then all of another -- camera drift between calls makes angles mismatch
- **FSD and variant switching**: With FSD enabled, variant switches update prim data but the viewport renders stale geometry. `set_variant_selection` detects FSD and applies the activation-toggle workaround only when needed. With FSD off, no workaround is needed
- **SetActive auto-framing**: `prim.SetActive(False/True)` can trigger viewport auto-framing, moving the camera unexpectedly. Avoid using it in `execute_script` unless necessary
- **stage.Flush() does not exist**: `stage.Flush()` is not a valid OpenUSD API. Don't call it. Use `stage.Reload()` to reload layers from disk (but it won't fix in-memory variant rendering)
- **Variant-internal edits**: `UsdShade.MaterialBindingAPI.Bind()` and similar Usd-level APIs write to the session/root layer, even inside a variant edit context. Use `Sdf` layer API for variant-internal edits
- **Anonymous stages**: Anonymous stages (`anon:...`) mean no file is loaded. Use `execute_script` with `stage.GetRootLayer().realPath` to check the stage URL before operating
- **Windows file paths**: In `execute_script`, use raw strings (`r"D:\3d\file.usda"`) or forward slashes (`"D:/3d/file.usda"`) -- backslashes without raw strings cause escape sequence issues
- **Large scene loading**: Datacenter-scale scenes with PointInstancer can take 15+ minutes to load. Don't assume the stage is ready immediately after opening
- **Black captures**: If `capture_viewport` or `inspect_prim` returns a WARNING about an all-black image, the scene likely has no lights. Use `viewport_light("get")` to check, then `viewport_light("set_camera_light", enabled=true)` to enable the camera light (a non-destructive fill light that follows the camera). Recapture after enabling. For best results, add proper scene lights with `create_prim` (e.g., `DistantLight`, `DomeLight`)
- **Scene lighting**: Kit 105.1+ has no default stage light. Scenes without lights render black. Use `viewport_light` to diagnose and toggle the camera light, or add lights via `create_prim`
- **Procedural vs Mesh geometry**: `UsdGeom.Cube`, `Sphere`, `Cylinder` etc. are implicit surfaces, not `UsdGeom.Mesh`. `get_mesh_stats` returns 0 faces for these. Only actual `UsdGeom.Mesh` prims (e.g., robot parts, imported models) have countable geometry
- **sim_control("stop") resets stage**: Stop resets the scene to its pre-play state. Use `sim_control("pause")` if you need to inspect mid-simulation state (positions, joint values, etc.)
- **Raycast requires playing sim**: `raycast()` uses PhysX which only works when the simulation is playing or has been played. If you get "No hit" unexpectedly, ensure you've called `sim_control("play")` first
