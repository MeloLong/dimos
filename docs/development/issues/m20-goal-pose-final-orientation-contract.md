# Issue: M20 Goal Pose And Final Orientation Contract

- Status: Open / design ready for implementation
- Priority: Medium for current click-to-go; High before docking, inspection, or tasks requiring a terminal heading
- Scope: m20_simple_nav goal ingress, planning contract, and terminal orientation behavior
- Affected modules: Viewer bridges, ReplanningAStarPlanner, GlobalPlanner, path resampling, and LocalPlanner
- Baseline inspected: MeloLong/dimos feat/wd/m20 at 8cdbbfa1
- Discovered on: 2026-07-15

## Summary

The M20 navigation planner accepts a full geometry_msgs.PoseStamped goal, so
the internal path and controller can already support a requested terminal yaw.
However, both interactive viewers currently submit position-only clicks. They
create an identity quaternion and therefore do not provide a way to request a
final heading.

There is also a contract ambiguity: the path resampler interprets the identity
quaternion (0, 0, 0, 1) as "final heading unspecified" and replaces it with
the final path tangent. Consequently, a caller cannot explicitly request
yaw = 0 rad without it being mistaken for an unspecified heading.

The solution is not a planner replacement. Preserve position-only click
compatibility, make final-heading intent explicit, and reuse PoseStamped as
the pose payload.

## Current Goal Data Flow

| Ingress | Current payload | Planner result |
| --- | --- | --- |
| Desktop dimos-viewer / Rerun WebSocket | JSON click: type, x, y, z, entity_path, timestamp_ms | Converted to PointStamped, then to PoseStamped with identity orientation. No terminal yaw can be requested. |
| Web Viewer / Command Center | Socket.IO click(position), position = [x, y] | Creates PoseStamped(position=(x, y, 0), orientation=(0,0,0,1), frame_id=world). No terminal yaw can be requested. |
| Programmatic publisher, RPC, patrol, agent | goal_request, target, or set_goal(PoseStamped) | A full pose can be supplied: position (x,y,z), quaternion (qx,qy,qz,qw), timestamp, and frame_id. |

ReplanningAStarPlanner subscribes to both goal_request: PoseStamped and
clicked_point: PointStamped. The point path is explicitly converted to an
identity-orientation pose. GlobalPlanner stores the full pose, but safe-goal
search and A* use only its position. It then calls:

    smooth_resample_path(path, current_goal, 0.1)

The resampler derives intermediate headings from the path tangent. For the
last pose, it uses supplied orientation only when that orientation is not the
identity quaternion. LocalPlanner enters final_rotation inside the 0.2 m
goal-position tolerance and rotates to the last path pose's yaw.

## Problems

### 1. Interactive Goal Inputs Strip Final Heading

**Severity: Medium. Capability and task-completion gap.**

Neither interactive UI has a heading-selection control or a pose-goal event.
The user can select where the robot stops but cannot select where it faces.

**Impact:** docking, camera inspection, manipulation approach, and corridor
exit tasks cannot state their desired terminal orientation. Users must bypass
the normal viewer path to exercise the existing pose capability.

### 2. Identity Quaternion Is an Ambiguous Sentinel

**Severity: High. Goal-contract correctness defect.**

The identity quaternion is a valid physical orientation: it represents zero
roll, pitch, and yaw. Current path resampling treats it as absent. An explicit
east-facing / yaw=0 goal instead becomes "finish along the path tangent".

**Impact:** terminal yaw zero cannot be reliably requested, and producer
behavior depends on a hidden convention rather than the message type.

### 3. Goal Frame Is Carried But Not Validated or Transformed

**Severity: Medium. Integration and safety boundary gap.**

PoseStamped.frame_id is serialized, but this M20 navigation path does not
transform a received goal at planner ingress. The goal must already use the
same map/world frame as SLAM odometry and the global costmap.

**Impact:** a camera-frame, base-frame, or misspelled frame goal can be planned
as if its coordinates were map coordinates.

### 4. Terminal Heading Precision Is Bounded by Existing Control Tuning

**Severity: Medium. Tracking-quality limitation.**

The final rotation is structurally present, but LocalPlanner tolerance is
0.35 rad (about 20 degrees) and M20 PController enforces a minimum angular
speed of 0.6 rad/s. Adding pose goals alone does not guarantee docking-grade
orientation precision.

## Target Contract

Keep PoseStamped as the pose payload. Do not introduce a second position or
quaternion encoding. Add explicit terminal-heading mode:

| Mode | Position | Final yaw behavior |
| --- | --- | --- |
| FOLLOW_PATH | (x,y,z) | Preserve legacy click behavior: derive final yaw from the final path segment. |
| FIXED | (x,y,z) plus quaternion | Honor the supplied quaternion exactly, including identity / yaw=0. |

The canonical long-term message can be a NavigationGoal with:

    pose: PoseStamped
    final_heading_mode: FOLLOW_PATH | FIXED

For the first implementation, a separate goal_pose_request: PoseStamped input
is sufficient to represent FIXED; existing clicked_point remains the
FOLLOW_PATH path. This avoids a breaking LCM migration while making intent
explicit at the planner boundary.

## Recommended Resolution

### P0: Establish Explicit Semantics Without Replacing the Planner

1. Keep clicked_point: PointStamped as the legacy location-only input.
2. Add goal_pose_request: In[PoseStamped] to ReplanningAStarPlanner and a
   matching RPC such as set_goal_pose.
3. Pass final_heading_required: bool into GlobalPlanner goal state: false for
   position-only clicks and true for goal_pose_request.
4. Change smooth_resample_path to accept an optional final orientation:
   None means derive final tangent; any quaternion, including identity, means
   preserve it exactly.
5. Keep existing goal_request and target stable until each producer is
   audited. Migrate each explicitly to positional or fixed-heading semantics;
   do not infer intent from quaternion values.

### P1: Add Viewer Support

**Web Viewer**

- Retain the existing click event for location-only goals.
- Add a backward-compatible goal_pose event with x, y, optional z, yaw,
  frame_id, and timestamp.
- Provide click-then-drag heading arrow or a yaw input control.
- Convert yaw to a normalized quaternion, then publish goal_pose_request.

**Desktop Rerun Viewer**

- Keep JSON click unchanged.
- Add a distinct JSON goal_pose event with position, yaw or quaternion,
  frame_id, and timestamp.
- Add goal_pose_request: Out[PoseStamped] to RerunWebSocketServer.
- Do not overload clicked_point: PointStamped cannot represent orientation and
  existing clients expect it to mean a position-only click.

### P2: Enforce Frame and Input Validity

1. Define accepted navigation frames, normally map or world after a clear
   alias policy.
2. Transform valid non-map goals through TF before planning, or reject them
   with a structured diagnostic until TF support exists.
3. Reject non-finite positions and invalid or non-normalizable quaternions.
4. Log goal source, position, final-heading mode, yaw, frame, and revision on
   every goal replacement.

### P3: Tune And Verify Terminal Tracking

- Make final orientation tolerance and low-speed angular behavior configurable.
- Test measured M20/MuJoCo angular response before tightening 0.35 rad.
- Align this tuning with planned speed-optimizer and terminal-braking work;
  do not hide terminal position overshoot by only changing orientation logic.

## Acceptance Tests

1. A legacy desktop or web click preserves current behavior and ends aligned
   with the final path segment.
2. A goal_pose_request at yaw=0 ends facing zero yaw, not the path tangent.
3. Repeat for +90 and -90 degree yaw; verify path terminal orientation,
   final-rotation commands, and goal_reached timing.
4. Replanning after an obstacle or goal replacement preserves fixed yaw.
5. If safe-goal search moves the final position, log the displacement and apply
   requested yaw at the accepted safe point.
6. A mismatched or unsupported frame is transformed or rejected; it must never
   silently be interpreted as map coordinates.
7. Existing click, goal, patrol, and agent tests pass with explicit heading mode.

## Relevant Files

- dimos/robot/deeprobotics/m20/nav/m20_simple_nav.py
- dimos/navigation/replanning_a_star/module.py
- dimos/navigation/replanning_a_star/global_planner.py
- dimos/mapping/occupancy/path_resampling.py
- dimos/navigation/replanning_a_star/local_planner.py
- dimos/navigation/replanning_a_star/controllers.py
- dimos/visualization/rerun/websocket_server.py
- dimos/web/websocket_vis/websocket_vis_module.py
- dimos/msgs/geometry_msgs/PoseStamped.py
- dimos/msgs/geometry_msgs/PointStamped.py
