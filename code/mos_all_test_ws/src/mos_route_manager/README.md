# mos_route_manager

`mos_route_manager` is the only global business-state node for the treatment-cart
workflow. It owns patrol goals, map-frame scan zones, fullness result correlation,
the dirty-cart delivery/clean-cart replacement sequence, and safe fault holding.
It does not open cameras, infer grasp points, or issue private SDK calls.

## Runtime ownership

- Only this package owns the `/navigate_to_pose` Nav2 Action client.
- Only the unified `mos_arm_controller` owns `MosController()` and provides both
  `/mos_arm_controller/run_task` and `/mos_chassis_controller/run_chassis`.
- Do not start the standalone `mos_chassis_controller` hardware node in formal
  operation.
- Do not start `lumos_nav/cmd_vel_to_base.py` in formal operation. Nav2 publishes
  `/cmd_vel`, while `mos_arm_controller` is the only node allowed to write that
  velocity to the physical chassis.
- `mos_cart_fullness` remains disabled outside the active scan or verification
  context.

## Configuration

`config/route_manager.yaml` contains the route names and map-frame scan corridors.
The named goal poses themselves are read from `goal_file`, defaulting to
`/opt/slam/config/target_goals.yaml`. It must provide these named poses:

```text
waypoint_left, waypoint_left_end, waypoint_right, waypoint_right_end,
grasp_a, dirty_car, clean_car
```

The supported goal-file layouts are either:

```yaml
goals:
  grasp_a:
    pose:
      position: {x: 1.0, y: 2.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

or a direct mapping where each goal has `position` and `orientation`.

## Start safely

Start the supporting nodes first: one `mos_3d_object`, `mos_cart_fullness`,
`mos_grasp_selector`, `mos_grasp_pipeline`, and the unified hardware controller.
Then launch this node with patrol still disabled:

```bash
ros2 launch mos_route_manager route_manager.launch.py auto_start:=false
```

After `/localization` is map-frame, the Nav2 `/navigate_to_pose` Action server is
ready, and the required services are available, start patrol explicitly:

```bash
ros2 service call /mos_route_manager/start std_srvs/srv/Trigger "{}"
```

Observe state without commanding motion:

```bash
ros2 topic echo /mos_route_manager/status
```

Any pipeline, place, Nav2 Action, or fullness-service failure enters
`fault_hold` and disables fullness. The manager does not resume automatically.
On fault the manager disables `/cmd_vel` forwarding in `mos_arm_controller` and
cancels the active Nav2 Action. Validate the robot's emergency-stop procedure
before enabling autonomous patrol.
After physical inspection, reset only to the idle state and manually start again:

```bash
ros2 service call /mos_route_manager/reset_fault std_srvs/srv/Trigger "{}"
```
