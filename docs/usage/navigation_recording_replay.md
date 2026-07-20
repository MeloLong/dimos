# Navigation Recording And Replay

DimOS navigation runs can produce two complementary artifacts:

| Artifact | Purpose | Capture path | Replay path |
| --- | --- | --- | --- |
| Rerun `.rrd` | Visual diagnosis, sensor inspection, and evidence | Rerun gRPC server with `--save` | Rerun native or web viewer |
| memory2 SQLite `.db` | Structured, queryable module streams | `nav-record` module | Render to `.rrd` or consume with `store.replay()` |
| Timed pickle directory | Legacy sensor-test fixture | `TimedSensorStorage` | `TimedSensorReplay` |

An `.rrd` is a visual event log, not the primary training dataset. A `.db`
only contains streams actually connected to `nav-record`; always inspect its
stream registry and counts before using it for evaluation or training.

## Rerun RRD Capture

Start a Rerun gRPC server before DimOS. `--save` streams incoming Rerun events
to the named file; stop the server cleanly with `Ctrl-C` after stopping DimOS
so the recording is finalized.

```bash
UV=uv
if ! command -v uv >/dev/null 2>&1 && [ -x "$HOME/.local/bin/uv" ]; then
  UV="$HOME/.local/bin/uv"
fi

mkdir -p /public/M20_dimos
"$UV" run --no-sync rerun --serve-grpc --port 9877 \
  --server-memory-limit 15GB \
  --save "/public/M20_dimos/m20_$(date +%Y%m%d_%H%M%S).rrd"
```

In a second terminal on the same host, run the navigation blueprint:

```bash
"$UV" run --no-sync dimos --rerun-open none run m20-simple-nav-sim
```

The M20 Rerun bridge targets `rerun+http://127.0.0.1:9877/proxy`. When that
port already has a Rerun server, the bridge connects to it rather than opening
a competing server. The RRD receives Rerun-convertible bridge events, such as
RGB, SLAM point clouds, maps, TF, and planner visuals when those streams are
enabled. It is not a raw transport capture of every DimOS topic.

`--server-memory-limit` controls live-server buffering, not a desired output
file size. Choose a value that leaves enough RAM for simulation and check disk
space before long runs.

### Validate And Replay An RRD

```bash
"$UV" run --no-sync rerun rrd verify /public/M20_dimos/run.rrd
"$UV" run --no-sync rerun rrd stats --no-decode /public/M20_dimos/run.rrd

# Native viewer
"$UV" run --no-sync rerun /public/M20_dimos/run.rrd --memory-limit 8GB

# Remote or headless host
"$UV" run --no-sync rerun /public/M20_dimos/run.rrd \
  --web-viewer --bind 0.0.0.0 --web-viewer-port 9090 \
  --server-memory-limit 2GB --memory-limit 4GB
```

Open `http://<host-ip>:9090` for the web viewer. Do not use `--follow` for a
completed RRD. For entity inventory, filtering, or quantitative comparisons,
use the `dimos-rrd-replay-analysis` workflow.

## SQLite NavRecord Capture

`nav-record` is an optional module composed with a navigation blueprint. Its
configuration key is `navrecord`, not `nav_record`.

```bash
mkdir -p /public/M20_dimos/db
"$UV" run --no-sync dimos --rerun-open none run m20-simple-nav-sim nav-record \
  --option "navrecord.db_path=/public/M20_dimos/db/m20_$(date +%Y%m%d_%H%M%S).db"
```

The recorder opens a memory2 SQLite store and writes each connected `In` port
to a named stream. It preserves timestamps and, when TF is available, pose
anchors. The database is structured data suitable for stream queries and
replay-driven tooling, unlike the viewer-oriented RRD.

The generic recorder does not subscribe by wildcard. In the current
`m20-simple-nav-sim` wiring, it automatically records TF and `global_map`,
but does not alias `dimos/slam_odom` to `odometry` or
`dimos/slam_aligned_points` to `registered_scan`; it also has no RGB input.
Treat that artifact as partial until an M20-specific recorder composition adds
those mappings.

### Inspect And Render A DB

List registered streams before relying on the dataset:

```bash
"$UV" run --no-sync python - /public/M20_dimos/db/run.db <<'PY'
import sqlite3
import sys

conn = sqlite3.connect(f"file:{sys.argv[1]}?mode=ro", uri=True)
for (name,) in conn.execute("SELECT name FROM _streams ORDER BY name"):
    print(name)
conn.close()
PY
```

Render every Rerun-convertible stream in the database to a visual replay:

```bash
"$UV" run --no-sync dimos mem rerun /public/M20_dimos/db/run.db \
  --out /public/M20_dimos/db/run-from-db.rrd --no-gui
"$UV" run --no-sync rerun rrd verify /public/M20_dimos/db/run-from-db.rrd
"$UV" run --no-sync rerun /public/M20_dimos/db/run-from-db.rrd
```

This converts recorded observations to an RRD; it does not relaunch a robot
or replay control into hardware. For programmatic, time-aligned stream replay,
open the store and subscribe to its replay streams before the shared clock
advances:

```python
from dimos.memory2.cli.dataset import open_dataset

store = open_dataset("/public/M20_dimos/db/run.db")
replay = store.replay(speed=1.0, seek=0.0, duration=30.0)
replay.streams.global_map.observable().subscribe(handle_map)
replay.streams.odometry.observable().subscribe(handle_odometry)
```

A hardware-free full navigation rerun needs a dedicated replay connection or
blueprint that republishes the selected streams and intentionally disconnects
real robot actuation. There is no safe generic command that replays a NavRecord
DB into a live robot.

## Per-Run Metadata

Keep the RRD and DB with a lightweight manifest that records the scenario or
map, goal, branch and commit, resolved configuration, start and end times,
outcome, and relevant DimOS logs. Store large artifacts outside normal Git
history.
