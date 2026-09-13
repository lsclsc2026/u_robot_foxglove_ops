ros2 topic pub --once \
  /cart_fullness/route_signal \
  std_msgs/msg/String \
  'data: "{\"decision\":\"full\",\"check_id\":\"manual_full_001\",\"waypoint\":\"patrol_point_a\",\"signal\":\"continue_to_next_waypoint_for_grasp\",\"route_step_delta\":1}"'

ros2 topic pub --once \
  /cart_fullness/status \
  std_msgs/msg/String \
  'data: "{\"state\":\"checking\",\"decision\":\"no_cart\",\"check_id\":\"scan_a_001\",\"zone_id\":\"scan_a\",\"cart_id\":\"cart_a\",\"camera\":\"left\"}"'

ros2 topic pub --once \
  /vision/grasp_tasks mos_grasp_selector/msg/GraspTaskArray "{
  header: {frame_id: 'body_link'},
  tasks: [{
    header: {frame_id: 'body_link'},
    task_name: 'grasp',
    grasp_mode: 'dual_arm_sync',
    target_frame: 'body_link',
    ready: true,
    quality: 'manual',
    reason: 'manual_topic_test',
    trigger_class_id: 6,
    trigger_class_name: 'handle',
    trigger_object_id: 1,
    trigger_score: 1.0,
    target_class_id: 6,
    target_class_name: 'handle',
    required_points: 2,
    points: [
      {
        header: {frame_id: 'body_link'},
        role: 'left_handle',
        rank: 0,
        class_id: 6,
        class_name: 'handle',
        source_object_id: 1,
        source_score: 1.0,
        point: {header: {frame_id: 'body_link'}, point: {x: 0.75, y: 0.25, z: -0.55}},
        distance_m: 0.0
      },
      {
        header: {frame_id: 'body_link'},
        role: 'right_handle',
        rank: 0,
        class_id: 6,
        class_name: 'handle',
        source_object_id: 2,
        source_score: 1.0,
        point: {header: {frame_id: 'body_link'}, point: {x: 0.75, y: -0.25, z: -0.55}},
        distance_m: 0.0
      }
    ]
  }]
}"



ros2 topic pub --once \
  /cart_fullness/status \
  std_msgs/msg/String \
  'data: "{\"state\":\"checking\",\"decision\":\"no_cart\",\"check_id\":\"scan_a_001\",\"zone_id\":\"scan_a\",\"cart_id\":\"cart_a\",\"camera\":\"left\"}"'