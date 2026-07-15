# DimOS Memory

`memory2` is DimOS's persistent observation store and streaming layer. It turns
live robot streams into queryable datasets while keeping timestamps, poses,
tags, and encoded payloads together.

Use it when you need to record a run, inspect a dataset, render it in Rerun, or
build a pipeline that processes historical and live observations with the same
API.

## Start Here

For a navigation run, recording is composed with the robot blueprint. The
`NavRecord` module records the streams that are actually connected to it:

```bash
mkdir -p recordings
dimos run m20-dan-nav nav-record \
  --option navrecord.db_path=recordings/m20-dan-$(date +%Y%m%d_%H%M%S).db
```

Stop the run gracefully with `Ctrl-C` so the SQLite database is closed and
flushed. The default `NavRecord` path is `nav_recording.db` in the DimOS
project root. Existing output is backed up by default instead of silently
overwritten.

Render a finished recording into a portable Rerun recording:

```bash
dimos mem rerun recordings/m20-dan-20260715_170000.db \
  --out recordings/m20-dan-20260715_170000.rrd \
  --no-gui
```

Omit `--no-gui` to open the Rerun viewer after writing the `.rrd` file. Add
`--seconds N` to render only the first `N` seconds, or `--root session_name` to
place all entities below a common Rerun path.

## Mental Model

```text
Robot modules -> DimOS streams -> Recorder -> SQLite dataset -> query / render
                                      |                         |
                                      +-- timestamps, poses,     +-- .rrd export
                                          tags, encoded payloads
```

The core objects are:

| Object | Responsibility |
| --- | --- |
| `Store` | Owns named streams and their storage backend. |
| `Stream` | Lazy query, transform, and iteration API for one observation stream. |
| `Recorder` | A DimOS module that persists its connected `In` ports to a store. |
| `SqliteStore` | Persistent SQLite implementation used for recordings. |
| `MemoryStore` | In-memory store for experiments and tests. |
| `NullStore` | Live-only, constant-memory store with no history. |

An observation contains payload data plus timestamp, optional pose, frame ID,
and tags. Payloads are encoded automatically: images use JPEG compression,
DimOS LCM messages use LCM encoding, and other values use pickle as a fallback.

## Recording a Blueprint

There is no global recorder that captures every process automatically. Recording
is explicit: add a recorder module to the blueprint or compose an existing
recorder on the CLI. A recorder only persists streams that are wired to one of
its `In` ports.

The M20 command above combines two registered blueprints:

```text
m20-dan-nav + nav-record
```

`NavRecord` is intended for navigation data such as paths, goals, velocity
commands, odometry, and connected map streams. It is not a raw camera or lidar
packet recorder. Add a dedicated recorder when raw sensor packets or a custom
set of streams are required.

### Add a Custom Recorder

Define the inputs to persist, then compose the recorder with the producer
blueprint. Matching stream name and type let `autoconnect()` wire the recorder.

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In
from dimos.memory2.module import Recorder
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image


class SessionRecorder(Recorder):
    color_image: In[Image]
    odometry: In[Odometry]


recorded_robot = autoconnect(
    robot_blueprint,
    SessionRecorder.blueprint(db_path="recordings/session.db"),
)
```

Use a specific recorder class instead of an unbounded catch-all recorder. This
makes the recorded contract visible, avoids accidental high-bandwidth capture,
and keeps replay datasets predictable.

## Inspect a Dataset from Python

```python
from dimos.memory2.store.sqlite import SqliteStore

store = SqliteStore(path="recordings/m20-dan-20260715_170000.db")

for name, stream in store.streams.items():
    print(name, stream.summary())

recent_odom = store.streams.odometry.order_by("ts", desc=True).limit(10).to_list()
```

Queries are lazy until a terminal operation such as `.to_list()`, `.first()`,
`.count()`, or `.drain()` consumes them. For a live pipeline, use
`.drain_thread()` rather than a materializing operation:

```python
handle = (
    store.streams.color_image.live()
    .transform(process_frame)
    .save(store.stream("processed_image", Image))
    .drain_thread()
)
```

Live streams are unbounded. Operations that require the whole input, including
sorting and vector search after a live transform, are intentionally rejected.
Query the stored stream instead, or bound the input with `.limit()`.

## Storage and Operations

- A recording is a SQLite database, not a video file. Use `dimos mem rerun` to
  create a `.rrd` file for Rerun playback and sharing.
- Stop recording cleanly before copying or archiving a database. SQLite may use
  WAL sidecar files while the recorder is active.
- Point clouds and images can consume disk space quickly. Choose the streams to
  record deliberately and monitor available disk space during long sessions.
- Runtime logs are separate from datasets. DimOS run logs live under
  `logs/<run-id>/main.jsonl`; they do not replace a `Recorder` dataset.
- Keep recording metadata with the database: robot model, software revision,
  transport configuration, test area, and operator notes are needed to make a
  dataset useful later.

## Further Reading

- [Introduction](intro.md): stores, streams, filters, transforms, and live queries.
- [Architecture](architecture.md): backend, observation, blob, vector, and notifier layers.
- [Streaming model](streaming.md): lazy, materializing, and terminal operations.
- [Store implementations](store/README.md): persistent and in-memory backends.
- [Codecs](codecs/README.md): payload encoding rules.
- [Memory capability guide](../../docs/capabilities/memory/index.md): visualization and analysis examples.

For a complete Chinese version of this guide, see [README.zh-CN.md](README.zh-CN.md).
