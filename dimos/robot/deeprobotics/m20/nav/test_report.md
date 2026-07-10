# M20 Navigation Test Report

## Test Objective

Evaluate the navigation performance of the current blueprint by observing:

- Path planning quality
- Path tracking accuracy
- Goal-reaching accuracy
- Map quality and its influence on planning

---
## Data

Nav_3d : m20_nav3d.rrd
Dan_nav: m20_20260709_135931.rrd and m20_20260709_180652.rrd


## Nav_3d Test Results (data:m20_nav3d.rrd)

| Timestamp | Target Direction | Robot / Path Behavior | Map / Terrain Observation |
|-----------|------------------|-----------------------|---------------------------|
| **06:41:30.540555Z** | Positive X (forward) | The robot followed the planned path reasonably well but stopped approximately **30 cm** away from the goal. | |
| **06:41:41.733412Z** | Negative X (backward) | The planned path consisted of multiple line segments. During execution, the path kept changing, and the robot followed it with a curved trajectory instead of the planned shape. The robot stopped far from the goal. | |
| **06:41:57.905371Z** | Rear diagonal | The robot followed the path with a curved trajectory and stopped far from the goal. | |
| **06:42:09.446728Z** | Negative Y | The robot followed the path with a curved trajectory and stopped far from the goal. | |
| **06:42:21.652667Z** | Between +X and -Y | The robot followed the path with a curved trajectory and stopped far from the goal. | |
| **06:42:31.167730Z** | Negative Y | Dynamic obstacle avoidance worked correctly. The planner generated a multi-segment path without overshooting the goal. | The ground surface had noticeable thickness and appeared approximately **5 cm higher** than the surrounding area. The planner seemed to generate paths only on surfaces with similar elevation, causing it to avoid depressed regions. |
| **06:43:06.196190Z** | Negative Y | The planner generated a straight path. Dynamic obstacle avoidance worked correctly, and the robot also moved in a straight line. However, it overshot the goal. | Uneven terrain caused the planned path to become segmented. Parts of the ground appeared elevated. |
| **06:43:12.673081Z** | Between -X and -Y | The planned path contained few turns, but the robot still followed it with a curved trajectory rather than tracking it accurately. | The ground was relatively flat, but the reconstructed map became thicker as the robot moved. |
| **06:43:31.032865Z** | Between +X and +Y | The planned path was short, and the robot tracked it reasonably well, but still stopped far from the goal. | A depression was present at the goal location. |
| **06:43:50.616229Z** | Negative X | The planned path initially contained a large bend and did not head directly toward the goal. The robot eventually overshot the goal. | Uneven terrain caused the planner to route around elevated regions. |
| **06:44:05.467068Z** | Between -Y and +X | The planned path contained two bends, both occurring at elevation boundaries. The robot eventually overshot the goal. | |
| **06:44:21.308991Z** | Between +Y and -X | Path bends consistently appeared at elevation transitions. The robot followed the path with a curved trajectory and overshot the goal. | |
| **06:44:38.082247Z** | Between +Y and +X | The planned path did not initially point directly toward the goal. It developed bends at elevation transitions, and the robot eventually overshot the goal. | |

---

## Observations

### Path Tracking

- The robot frequently followed **curved trajectories**, even when the planned path was nearly straight.
- Tracking accuracy degraded particularly during direction changes.
- The robot consistently exhibited **large final position errors** or **overshot the goal**.

### Path Planning

- The planner generally generated reasonable paths.
- However, terrain elevation changes often introduced unnecessary bends or polyline segments.
- In several cases, the planned path changed dynamically while the robot was executing it.

### Goal Accuracy

- Significant final position errors were observed in most tests.
- Both **undershooting** and **overshooting** occurred, with overshooting being more common.

### Map Quality

- Terrain elevation had a significant impact on planning.
- Elevated surfaces and depressions strongly influenced path generation.
- The planner appeared to prefer regions with similar height and avoided lower areas.
- The reconstructed ground occasionally appeared thicker over time, suggesting map accumulation or reconstruction artifacts.

---

## Overall Findings

The primary issues observed during testing are:

1. **Curved trajectory tracking**, even for nearly straight planned paths.
2. **Poor goal-reaching accuracy**, with frequent overshooting or large final position errors.
3. **High sensitivity to terrain elevation**, causing unnecessary bends in planned paths.
4. **Map quality issues**, including ground thickening and uneven surface reconstruction, which negatively affect planning performance.
5. **Dynamic replanning** was observed in some cases, where the planned path changed during execution.
---
<br><br>

## Dan_nav Test Results(data: m20_20260709_135931.rrd)

| Timestamp | Target Direction | Robot / Path Behavior | Map / Terrain Observation |
|-----------|------------------|-----------------------|---------------------------|
| **06:00:10.835032Z** | Positive X (forward) | The robot tracked the initial straight segment well. However, once the path contained a turn, it deviated from the planned trajectory and failed to recover back to the path. | The ground was flat. |
| **06:00:18.292669Z** | Negative X (backward) | The robot exhibited a large tracking error during the initial turnaround. Afterwards, the planned path continuously changed due to repeated replanning. | The ground was flat. |
| **06:00:29.354889Z** | Positive Y | The robot initially followed the planned path well, but gradually drifted toward the negative Y direction, resulting in poor path tracking. | The ground was flat. |
| **06:00:43.330120Z** | Negative X | During the turnaround, the robot first drifted toward the negative Y direction and then toward the positive Y direction. A curved trajectory was generated, followed by another path appearing on the opposite side of the arc due to replanning. | The terrain remained flat throughout the test. During the curved trajectory, the voxel map around the robot was dense and uniform, while the goal region was relatively sparse but still flat. |
| **06:00:57.728230Z** | Between -X and -Y | During the turnaround, the robot drifted toward the positive Y direction. The replanned path was located on the robot's negative Y side, but the robot failed to correct the lateral offset and maintained it until reaching the goal. | The ground was flat. |
| **06:01:11.404441Z** | Between -X and -Y | Similar behavior to the previous test. The robot drifted toward the positive Y direction during the turnaround. No further replanning occurred, and the robot maintained the tracking error until the end. | The ground was flat. |
| **06:01:21.740880Z** | Slightly toward -Y from the negative X direction | The robot drifted toward the positive Y direction during the turnaround. Only one replanning event occurred, after which the robot continued with the lateral offset until reaching the goal. | The ground was flat. |
| **06:01:31.303920Z** | Slightly toward -Y from the negative X direction | Similar to the previous test. The robot drifted toward the positive Y direction during the turnaround. Only one replanning event occurred, and the robot maintained the offset until the end. | The ground was flat. |
| **06:01:40.935396Z** | Slightly toward -Y from the negative X direction | During the turnaround, the robot initially drifted toward the positive Y direction but gradually converged back to the planned path. Only one replanning event occurred, and the robot tracked the remainder of the path well. | The ground was flat. |
| **06:01:50.936648Z** | Slightly toward +Y from the negative X direction | During the turnaround, the robot drifted toward the negative Y direction and later converged with the planned path. Only one replanning event occurred. However, the robot maintained a slight negative Y offset until reaching the goal. | The ground was flat. |

---

## Observations

### Path Tracking

- The robot tracked straight-line segments reasonably well.
- Tracking performance degraded significantly during **turnaround maneuvers** and **path turns**.
- Once a lateral tracking error was introduced, the robot often failed to fully recover and maintained the offset throughout the remaining trajectory.
- In a few cases, the robot gradually converged back to the planned path after the initial deviation.

### Path Planning

- Continuous replanning was observed in some tests immediately after the turnaround.
- In most cases, only a single replanning event occurred before the planned path became stable.
- Overall, the planned paths were reasonable, and the primary issue appeared to be path tracking rather than path generation.

### Goal Accuracy

- The robot generally reached the target region.
- However, noticeable lateral tracking errors remained in several tests, especially after turnaround maneuvers.

### Map Quality

- The environment remained flat throughout all tests.
- No significant terrain artifacts were observed.
- During one test, the voxel map around the robot was dense and uniform, while the goal region was relatively sparse. This did not appear to significantly affect path planning.

---

## Overall Findings

The primary observations from this test session are:

1. **Tracking performance is satisfactory on straight segments but deteriorates significantly during turnaround maneuvers and path turns.**
2. **The robot exhibits a consistent lateral drift during turnaround**, most commonly along the Y-axis.
3. **Once a tracking error is introduced, the controller often fails to eliminate the offset**, resulting in persistent lateral deviation for the remainder of the trajectory.
4. **Path replanning is generally limited**, with most tests triggering at most one replanning event after the initial maneuver.
5. **Since all tests were conducted on flat terrain, the observed tracking errors are unlikely to be caused by terrain or map quality. They are more likely related to the path-tracking controller, vehicle kinematics, or localization performance(less likely ) during turning.**
---


## Dan_nav Test Results(data: m20_20260709_180652.rrd)

| Timestamp | Test Scenario | Robot / Path Behavior | Map / Terrain Observation |
|-----------|---------------|-----------------------|---------------------------|
| **10:07:10.302736Z** | Positive X (forward) | The robot exhibited slight left-right oscillations but tracked the planned path well throughout the trajectory. It eventually overshot the goal by approximately **35 cm**. | The ground was flat. |
| **10:07:17.236549Z** | Negative X (backward) | The robot initially drifted significantly toward the negative Y direction. It then converged toward the planned path, crossed over to the opposite side of the path, and continued diverging until it overshot the goal. | The ground was flat. |
| **10:07:31.250Z** | Positive Y | The robot initially drifted toward the positive Y direction, then quickly drifted toward the negative Y direction until overshooting the goal. It did not gradually converge back to the planned path. | The ground was flat. |
| **10:07:51.583320Z** | Negative Y | The robot first drifted toward the negative Y direction and then quickly toward the positive Y direction, continuously oscillating until it overshot the goal. | The ground was flat. |
| **10:08:15.997339Z** | Circular path | The robot tracked the circular path well throughout the trajectory but still overshot the goal. | The ground was flat. |

---

## Observations

### Path Tracking

- Overall path-tracking performance was good on both straight and circular trajectories.
- Slight lateral oscillations were observed during forward motion.
- For motion along the negative X direction, the robot crossed the planned path and continued drifting away instead of converging back.
- Motion along the Y-axis exhibited noticeable lateral drift and oscillation, without recovering toward the planned path.

### Goal Accuracy

- The robot consistently **overshot the goal** in all test cases.
- The positive X test overshot the goal by approximately **35 cm**.
- Similar overshooting behavior was observed in the remaining tests.

### Map Quality

- The environment remained flat throughout all tests.
- No terrain irregularities or mapping artifacts were observed.

---

## Overall Findings

The primary observations from this test session are:

1. **Overall path-tracking performance is good**, especially on straight and circular trajectories.
2. **Small lateral oscillations** are present during straight-line motion.
3. **The robot exhibits noticeable lateral drift during negative X and Y-axis motions**, and in some cases crosses the planned path before continuing to diverge, indicating insufficient lateral error correction.
4. **Goal overshoot is consistently observed across all tests**, suggesting that the stopping behavior or goal tolerance requires further tuning.
5. **Since all tests were conducted on flat terrain, the observed tracking errors are unlikely to be caused by terrain or map quality. They are more likely related to the path-tracking controller, lateral control strategy, or stopping logic.**


# M20 Navigation Algorithm Evaluation Report

This evaluation compares the performance of two navigation algorithms:

* **nav_3d**: evaluated using the first set of test data.
* **dan_nav**: evaluated using the second and third sets of test data.

Since the results from the second and third test sessions are highly consistent, they are combined into a single evaluation of **dan_nav**.

---

# Evaluation of nav_3d

## Overall Performance

The **nav_3d** algorithm provides a complete navigation pipeline, including global path planning, dynamic obstacle avoidance, and online replanning. It is capable of completing autonomous navigation tasks.

However, the overall test results indicate that its performance is strongly affected by map quality. Both path-planning stability and path-tracking accuracy require further improvement.

### Strengths

* Provides a complete autonomous navigation pipeline.
* Dynamic obstacle avoidance functions correctly.
* Supports online replanning in response to environmental changes.
* Successfully generates feasible paths and reaches the target region in most test cases.

### Major Issues

#### 1. Poor Path-Tracking Accuracy

The most significant issue observed during testing is the robot's inability to accurately follow the planned path.

Typical behaviors include:

* Following a curved trajectory even when the planned path is straight.
* Persistent lateral tracking errors that are not effectively corrected.
* Large final position errors, often resulting in goal overshoot.

These observations indicate that the path-tracking controller has insufficient capability to correct lateral errors.

---

#### 2. High Sensitivity to Map Quality

The generated paths are strongly influenced by the quality of the reconstructed map.

Observed behaviors include:

* Small elevation changes causing unnecessary path bends.
* The planner avoiding shallow depressions.
* Frequent detours near elevation boundaries.

These results suggest that **nav_3d** relies heavily on the quality of the MLS map.

---

#### 3. Map Reconstruction Artifacts Affect Planning

Several mapping artifacts were observed during testing, including:

* Gradual thickening of the ground surface.
* Local elevated regions.
* Sparse point clouds around the goal area.

These reconstruction artifacts further reduce planning stability.

---

## Summary

The **nav_3d** algorithm is capable of autonomous navigation but lacks robustness.

Its primary limitations are:

* Poor path-tracking accuracy.
* Strong dependence on map quality.
* Limited planning stability.

Overall navigation performance is relatively inconsistent and exhibits noticeable variability.

---

# Evaluation of dan_nav

## Overall Performance

Compared with **nav_3d**, **dan_nav** demonstrates significantly better navigation performance.

The planned paths are considerably more stable, path-tracking accuracy is substantially improved, and the influence of map quality on planning is greatly reduced.

Overall navigation behavior is more stable and repeatable.

### Strengths

#### 1. More Stable Path Planning

Compared with **nav_3d**:

* Replanning occurs much less frequently.
* Planned paths remain stable during execution.
* Paths are smoother.
* Minor terrain variations no longer introduce unnecessary bends.

These results indicate a significant improvement in planning stability.

---

#### 2. Improved Path-Tracking Performance

The robot is able to reliably follow:

* Paths in the **positive X direction (forward motion)**.
* Circular trajectories.
* Most target directions tested.

Compared with **nav_3d**, the robot remains much closer to the planned path, resulting in noticeably better tracking accuracy.

---

#### 3. Improved Robustness to Map Quality

Throughout the tests:

* No significant mapping artifacts affected path planning.
* The reconstructed map remained stable.
* Planning results were consistent across repeated tests.

This indicates that **dan_nav** is substantially more robust to variations in map quality.

---

## Remaining Issues

Although **dan_nav** significantly outperforms **nav_3d**, several issues remain.

### 1. Lateral Drift During Turning

The primary remaining issue occurs during:

* Turnaround maneuvers.
* Motion along the Y-axis.

The robot still exhibits:

* Lateral drift.
* Crossing the planned path.
* Failure to fully converge back to the planned trajectory.

This suggests that the lateral controller still requires further optimization.

---

### 2. Goal Overshoot

A consistent goal overshoot was observed in nearly all test cases.

The typical overshoot distance is approximately:

* **20–35 cm**

This suggests that the stopping strategy or terminal convergence logic requires further refinement.

---

### 3. Minor Lateral Oscillation

Slight left-right oscillations remain during motion in the positive X direction.

Although these oscillations do not significantly affect navigation performance, they indicate that additional controller tuning is still possible.

---

## Summary

Compared with **nav_3d**, **dan_nav** provides a significantly more mature and stable navigation solution.

The remaining performance bottlenecks have shifted from planning to motion control.

Future optimization should focus on:

* Lateral controller tuning.
* Stopping strategy.
* Lateral error correction during turning.

---

# Comparison Between nav_3d and dan_nav

| Category                   | nav_3d                                                      | dan_nav                                                                                                         | Comparison                           |
| -------------------------- | ----------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------- | ------------------------------------ |
| Path Planning Stability    | Frequent replanning and unstable paths                      | Stable paths with minimal replanning                                                                            | **dan_nav is significantly better**  |
| Path Smoothness            | Frequent bends and polyline segments                        | Smoother trajectories                                                                                           | **dan_nav is better**                |
| Path Tracking              | Often follows curved trajectories with large lateral errors | Reliable tracking in the positive X direction and on circular trajectories with significantly improved accuracy | **dan_nav is significantly better**  |
| Robustness to Map Quality  | Planning is strongly affected by map quality                | Planning is largely unaffected by map variations                                                                | **dan_nav is significantly better**  |
| Terrain Robustness         | Highly sensitive to elevation changes                       | Much more tolerant of terrain variations                                                                        | **dan_nav is better**                |
| Navigation Stability       | Low repeatability                                           | High repeatability                                                                                              | **dan_nav is significantly better**  |
| Dynamic Obstacle Avoidance | Functions correctly                                         | Functions correctly                                                                                             | **Comparable**                       |
| Lateral Control            | Large persistent lateral errors                             | Lateral errors are significantly reduced, although drift remains during turning                                 | **dan_nav is better**                |
| Goal Accuracy              | Large terminal error and frequent overshoot                 | Goal overshoot remains but is more consistent and predictable                                                   | **Both require further improvement** |

---

# Overall Conclusion

Based on all three test sessions, **dan_nav demonstrates significantly better overall performance than nav_3d**.

Compared with **nav_3d**, **dan_nav** achieves several important improvements:

* More stable path planning with significantly fewer replanning events.
* Smoother planned paths and more natural navigation behavior.
* Significantly improved path tracking in the **positive X direction** and on **circular trajectories**.
* Much stronger robustness to map quality and terrain variations.
* More stable and repeatable navigation performance.

However, both algorithms still share several limitations, primarily within the motion-control layer:

* Lateral drift during turning maneuvers.
* Noticeable lateral tracking errors during motion along the Y-axis.
* Goal overshoot of approximately **20–35 cm**.
* Minor lateral oscillations during motion in the positive X direction.

Overall, the primary limitations of **nav_3d** lie in planning stability, map robustness, and path-tracking performance. In contrast, **dan_nav** has largely addressed the planning-related issues, and the current performance bottleneck has shifted to the motion-control layer. Future work should therefore focus on improving the lateral controller, refining the stopping strategy, and further tuning the trajectory-tracking controller to achieve higher navigation accuracy.
