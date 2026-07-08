# M20 Navigation Testing Plan

## Objective

The objective of this testing is to evaluate the performance of the current M20 navigation blueprints, collect recordings for offline analysis, and identify potential issues related to navigation, mapping, and latency.

The testing will focus on:

- Navigation performance
- Path tracking accuracy
- Map quality
- Rerun latency
- Overall system stability

---

# Day 1 — Blueprint Validation & Manual Navigation Testing

## 1. Validate Blueprints

Test the following blueprints:

### A. m20-nav-3d

Location:

```
dimos/robot/deeprobotics/m20/blueprints/basic.py
```

### B. m20-simple-nav

Location:

```
dimos/robot/deeprobotics/m20/nav/m20_simple_nav.py
```

Launch individually:

```bash
dimos run m20-nav-3d
```

```bash
dimos run m20-simple-nav
```

Verify:

- Blueprint starts successfully
- Camera data available
- LiDAR data available
- Rerun visualization works
- Navigation initializes correctly

Save startup logs.

---

## 2. Manual Navigation Recording

Select a safe, open, flat testing area.

Recommended:

- Free hangar near the office

Two operators are required.

### Operator A

Responsible for:

- Laptop
- Launching blueprint
- Recording
- Sending navigation goals

### Operator B

Responsible for:

- Remote controller
- Emergency stop
- Safety monitoring

---

### Recording Requirements

For each blueprint:

- Record at least one session
- Approximately 10 navigation goals
- Flat ground only
- No stairs
- No slopes

Record:

- Recording
- Log files

Observe:

- Successful path planning
- Robot reaches goal
- Path tracking quality
- Motion smoothness
- Navigation failures
- Unexpected stopping
- Control latency

---

## 3. Blueprint Selection

Compare both blueprints.

If

```
m20-nav-3d
```

performs similarly (or better) than

```
m20-simple-nav
```

continue future testing using

```
m20-nav-3d
```

Otherwise continue using

```
m20-simple-nav
```

---

# Day 2 — Automated Path Testing & Analysis

## 1. Automated Testing Blueprint

Create a testing blueprint based on the current working blueprint.

Requirements:

- Remove click-to-go interaction
- Remove MLSPlanner
- Inject predefined paths directly

Suggested trajectories:

### Straight Lines

- Forward 4 m
- Backward 4 m
- Left 4 m
- Right 4 m

### Curve

- One complete circle

Expected usage:

```bash
dimos run m20-test-lines
```

The robot should automatically execute all predefined trajectories.

---

## 2. Automated Recording

Record:

- Recording
- Log files

Observe:

- Path tracking accuracy
- Motion smoothness
- Velocity consistency
- Tracking errors

Immediately stop testing if abnormal behavior occurs.

---

## 3. Recording Analysis

Review all recordings.

Evaluate:

### Path Tracking

- Tracking accuracy
- Heading stability
- Turning performance

### Path Quality

Check whether:

- Straight paths remain straight
- Curved trajectories are smooth

### Mapping Quality

Inspect:

- Drift
- Missing regions
- Noise
- Map consistency

### Latency

Observe possible latency between:

- Navigation command
- Robot response
- Rerun visualization

Potential sources include:

- Network
- Software integration (DimOS / Deeprobotics)
- Zenoh
- Compute performance

---

# Deliverables

At the end of testing, the following should be available:

- Blueprint validation results
- Manual navigation recordings
- Automated trajectory recordings
- Log files
- Path tracking evaluation
- Mapping quality evaluation
- Latency observations
- Recommended blueprint for future development
