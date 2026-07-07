# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Rerun bridge for logging pubsub messages with to_rerun() methods."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import signal
import socket
import subprocess
import sys
import time
from typing import (
    Any,
    Protocol,
    TypeAlias,
    TypeGuard,
    cast,
    get_args,
    runtime_checkable,
)
from urllib.parse import urlparse

from reactivex.disposable import Disposable
import rerun as rr
from rerun._baseclasses import Archetype
import rerun.blueprint as rrb
from rerun.blueprint import Blueprint
from toolz import pipe  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
from dimos.protocol.pubsub.patterns import Glob, pattern_matches
from dimos.protocol.pubsub.spec import SubscribeAllCapable
from dimos.protocol.service.lcmservice import autoconf
from dimos.utils.generic import get_local_ips
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import (
    RERUN_ENABLE_WEB,
    RERUN_GRPC_PORT,
    RERUN_OPEN_DEFAULT,
    RERUN_WEB_VIEWER_PORT,
    RerunOpenOption,
)
from dimos.visualization.rerun.init import rerun_init

# TODO OUT visual annotations
#
# In the future it would be nice if modules can annotate their individual OUTs with (general or rerun specific)
# hints related to their visualization
#
# so stuff like color, update frequency etc (some Image needs to be rendered on the 3d floor like occupancy grid)
# some other image is an image to be streamed into a specific 2D view etc.
#
# To achieve this we'd feed a full blueprint into the rerun bridge.
#
# rerun bridge can then inspect all transports used, all modules with their outs,
# automatically spy an all the transports and read visualization hints
#
# Temporarily we are using these "sideloading" visual_override={} dict on the bridge
# to define custom visualizations for specific topics
#
# as well as pubsubs={} to specify which protocols to listen to.

# TODO better TF processing
#
# this is rerun bridge specific, rerun has a specific (better) way of handling TFs
# using entity path conventions, each of these nodes in a path are TF frames:
#
# /world/robot1/base_link/camera/optical
#
# While here since we are just listening on TFMessage messages which optionally contain
# just a subset of full TF tree we don't know the full tree structure to build full entity
# path for a transform being published
#
# This is easy to reconstruct but a service/tf.py already does this so should be integrated here
#
# we have decoupled entity paths and actual transforms (like ROS TF frames)
# https://rerun.io/docs/concepts/logging-and-ingestion/transforms
#
# tf#/world
# tf#/base_link
# tf#/camera
#
# In order to solve this, bridge needs to own it's own tf service
# and render it's tf tree into correct rerun entity paths

logger = setup_logger()

BlueprintFactory: TypeAlias = Callable[[], "Blueprint"]

RerunMulti: TypeAlias = "list[tuple[str, Archetype]]"
RerunData: TypeAlias = "Archetype | RerunMulti"


@dataclass
class _StreamDebugStats:
    window_start: float
    received: int = 0
    logged: int = 0
    throttled: int = 0
    frame_delta_count: int = 0
    frame_delta_sum_ms: float = 0.0
    frame_delta_last_ms: float | None = None
    frame_delta_max_ms: float | None = None
    convert_sum_ms: float = 0.0
    convert_max_ms: float = 0.0
    log_sum_ms: float = 0.0
    log_max_ms: float = 0.0
    sample: str = ""


def is_rerun_multi(data: Any) -> TypeGuard[RerunMulti]:
    """Check if data is a list of (entity_path, archetype) tuples."""
    return (
        isinstance(data, list)
        and bool(data)
        and isinstance(data[0], tuple)
        and len(data[0]) == 2
        and isinstance(data[0][0], str)
        and isinstance(data[0][1], Archetype)
    )


@runtime_checkable
class RerunConvertible(Protocol):
    """Protocol for messages that can be converted to Rerun data."""

    def to_rerun(self) -> RerunData: ...


def _hex_to_rgba(hex_color: str) -> int:
    """Convert '#RRGGBB' to a 0xRRGGBBAA int (fully opaque)."""
    h = hex_color.lstrip("#")
    if len(h) == 6:
        return int(h + "ff", 16)
    return int(h[:8], 16)


def _with_graph_tab(bp: Blueprint) -> Blueprint:
    """Add a Graph tab alongside the existing viewer layout without changing it."""

    root = bp.root_container
    return rrb.Blueprint(
        rrb.Tabs(
            root,
            rrb.GraphView(origin="blueprint", name="Graph"),
        ),
        auto_layout=bp.auto_layout,
        auto_views=bp.auto_views,
        collapse_panels=bp.collapse_panels,
    )


def _default_blueprint() -> Blueprint:
    """Default blueprint with black background and raised grid."""

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.5),
            ),
        ),
    )


def _default_pubsubs(config: Any = None) -> list[SubscribeAllCapable[Any, Any]]:
    """Select the pubsub backend based on the active transport.

    When transport is Zenoh, we listen on BOTH Zenoh and LCM because
    TF (transform frames) is currently hardcoded to LCM in the Module
    base class. Without LCM, the robot pose won't update in the viewer.
    """
    transport = getattr(config, "transport", None) or global_config.transport
    if transport == "zenoh":
        # Thread the parent's zenoh endpoints into the session: worker processes
        # don't see CLI overrides (e.g. --zenoh-connect) via the module-level
        # global_config singleton, so a bare Zenoh() would fall back to multicast
        # scouting and silently fail to reach a router across WiFi.
        zkwargs: dict[str, Any] = {}
        connect = getattr(config, "zenoh_connect", None)
        if connect:
            zkwargs["connect"] = [e.strip() for e in connect.split(",") if e.strip()]
        listen = getattr(config, "zenoh_listen", None)
        if listen:
            zkwargs["listen"] = [e.strip() for e in listen.split(",") if e.strip()]
        iface = getattr(config, "zenoh_iface", None)
        if iface:
            zkwargs["multicast_iface"] = iface
        # return [LCM()]
        return [Zenoh(**zkwargs), LCM()]
    return [LCM()]


def _resolve_pubsubs(config: Any) -> list[SubscribeAllCapable[Any, Any]]:
    """Return explicit pubsubs when truly overridden, else transport defaults.

    Older blueprints commonly passed ``pubsubs=[LCM()]`` as the effective
    default. Preserve the newer transport-driven behavior for that legacy
    value, but honor explicit non-default overrides such as custom backends.
    """
    fields_set: set[str] = cast("set[str]", getattr(config, "model_fields_set", set()))
    pubsubs = cast(
        "list[SubscribeAllCapable[Any, Any]] | None",
        getattr(config, "pubsubs", None),
    )
    if "pubsubs" in fields_set and pubsubs is not None:
        is_legacy_default = len(pubsubs) == 1 and isinstance(pubsubs[0], LCM)
        if not is_legacy_default:
            return pubsubs
    return _default_pubsubs(getattr(config, "g", config))


class Config(ModuleConfig):
    """Configuration for RerunBridgeModule.

    The pubsubs field is accepted for backwards compatibility. The legacy
    ``[LCM()]`` value is treated as the old default and replaced by the
    transport-driven runtime default. Explicit non-default overrides are still
    honored.
    """

    pubsubs: list[SubscribeAllCapable[Any, Any]] = field(default_factory=lambda: [LCM()])

    visual_override: dict[Glob | str, Callable[[Any], Archetype] | None] = field(
        default_factory=dict
    )
    static: dict[str, Callable[[Any], Archetype]] = field(default_factory=dict)
    max_hz: dict[str, float] = field(default_factory=dict)
    debug_stats: bool = False
    debug_stats_interval: float = 5.0
    debug_stats_entities: list[Glob | str] = field(default_factory=list)
    debug_low_fps_warn: dict[Glob | str, float] = field(default_factory=dict)
    use_message_timestamps: bool = True
    latest_only_entities: list[Glob | str] = field(default_factory=list)

    entity_prefix: str = "world"
    topic_to_entity: Callable[[Any], str] | None = None
    connect_url: str | None = None
    memory_limit: str = "25%"
    rerun_open: RerunOpenOption = RERUN_OPEN_DEFAULT
    rerun_web: bool = RERUN_ENABLE_WEB
    web_port: int = RERUN_WEB_VIEWER_PORT
    blueprint: BlueprintFactory | None = _default_blueprint


Config.model_rebuild(_types_namespace={"Archetype": Archetype, "Blueprint": Blueprint})


class RerunBridgeModule(Module):
    """Bridge that logs messages from pubsubs to Rerun.

    Spawns its own Rerun viewer and subscribes to all topics on each provided
    pubsub. Any message that has a to_rerun() method is automatically logged.

    Example:
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM

        lcm = LCM()
        bridge = RerunBridgeModule(pubsubs=[lcm])
        bridge.start()
        # All messages with to_rerun() are now logged to Rerun
        bridge.stop()
    """

    config: Config
    dedicated_worker = True
    _last_log: dict[str, float]

    # TODO this doesn't belong here, either hardcode it or put it to rerun bridge config
    GRAPH_VIZ_SCALE = 100.0
    MODULE_RADIUS = 20.0
    CHANNEL_RADIUS = 12.0

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._last_log = {}
        self._override_cache: dict[str, Callable[[Any], RerunData | None]] = {}
        self._frame_attached: dict[str, str] = {}
        self._debug_stats: dict[str, _StreamDebugStats] = {}
        self._debug_last_recv_time: dict[str, float] = {}

    @property
    def host(self) -> str:
        return self.config.g.rerun_host or self.config.g.listen_host

    def _visual_override_for_entity_path(
        self, entity_path: str
    ) -> Callable[[Any], RerunData | None]:
        """Return a composed visual override for the entity path.

        Chains matching overrides from config, ending with final_convert
        which handles .to_rerun() or passes through Archetypes. Cached per
        instance (not via ``lru_cache`` on a method, which would leak ``self``).
        """
        cached = self._override_cache.get(entity_path)
        if cached is not None:
            return cached

        matches = [
            fn
            for pattern, fn in self.config.visual_override.items()
            if pattern_matches(pattern, entity_path)
        ]

        # None means "suppress this topic entirely"
        if any(fn is None for fn in matches):

            def suppressed(msg: Any) -> RerunData | None:
                return None

            self._override_cache[entity_path] = suppressed
            return suppressed

        def final_convert(msg: Any) -> RerunData | None:
            if isinstance(msg, Archetype):
                return msg
            if is_rerun_multi(msg):
                return msg
            if isinstance(msg, RerunConvertible):
                return msg.to_rerun()
            return None

        # compose all converters
        def composed(msg: Any) -> RerunData | None:
            return cast("RerunData | None", pipe(msg, *matches, final_convert))

        self._override_cache[entity_path] = composed
        return composed

    def _get_entity_path(self, topic: Any) -> str:
        if self.config.topic_to_entity:
            return self.config.topic_to_entity(topic)

        topic_str = getattr(topic, "name", None) or str(topic)
        # Strip type suffix: LCM uses '#type', Zenoh embeds type as '/type' in key expr
        # but _key_expr_to_topic already parsed it into topic.topic, so use that.
        raw = getattr(topic, "topic", topic_str)
        if isinstance(raw, str):
            topic_str = raw
        topic_str = topic_str.split("#")[0]
        # Strip Zenoh key prefix (dimos/) to match LCM entity paths
        if topic_str.startswith("/dimos/"):
            topic_str = "/" + topic_str.removeprefix("/dimos/")
        elif topic_str.startswith("dimos/"):
            topic_str = "/" + topic_str.removeprefix("dimos/")
        return f"{self.config.entity_prefix}{topic_str}"

    def _debug_enabled_for_entity(self, entity_path: str) -> bool:
        if not self.config.debug_stats:
            return False
        if not self.config.debug_stats_entities:
            return True
        return any(
            pattern_matches(pattern, entity_path) for pattern in self.config.debug_stats_entities
        )

    def _debug_low_fps_threshold(self, entity_path: str) -> float | None:
        for pattern, threshold in self.config.debug_low_fps_warn.items():
            if pattern_matches(pattern, entity_path):
                return threshold
        return None

    def _latest_only_for_entity(self, entity_path: str) -> bool:
        return any(
            pattern_matches(pattern, entity_path) for pattern in self.config.latest_only_entities
        )

    def _debug_sample(self, msg: Any) -> str:
        try:
            if hasattr(msg, "pointcloud_tensor") or msg.__class__.__name__ == "PointCloud2":
                return f"points={len(msg)}"
            if hasattr(msg, "poses"):
                return f"poses={len(msg.poses)}"
            if hasattr(msg, "shape"):
                return f"shape={msg.shape}"
            if hasattr(msg, "transforms"):
                return f"transforms={len(msg.transforms)}"
        except Exception:
            return ""
        return ""

    def _debug_note_received(self, entity_path: str, msg: Any) -> None:
        if not self._debug_enabled_for_entity(entity_path):
            return
        now = time.monotonic()
        stats = self._debug_stats.get(entity_path)
        if stats is None:
            stats = _StreamDebugStats(window_start=now)
            self._debug_stats[entity_path] = stats

        prev_recv_time = self._debug_last_recv_time.get(entity_path)
        self._debug_last_recv_time[entity_path] = now
        if prev_recv_time is not None:
            frame_delta_ms = (now - prev_recv_time) * 1000.0
            stats.frame_delta_count += 1
            stats.frame_delta_sum_ms += frame_delta_ms
            stats.frame_delta_last_ms = frame_delta_ms
            stats.frame_delta_max_ms = (
                frame_delta_ms
                if stats.frame_delta_max_ms is None
                else max(stats.frame_delta_max_ms, frame_delta_ms)
            )

        stats.received += 1
        stats.sample = self._debug_sample(msg)

    def _debug_note_throttled(self, entity_path: str) -> None:
        stats = self._debug_stats.get(entity_path)
        if stats is not None:
            stats.throttled += 1
            self._debug_maybe_log(entity_path, time.monotonic())

    def _debug_note_logged(
        self,
        entity_path: str,
        convert_ms: float,
        log_ms: float,
    ) -> None:
        stats = self._debug_stats.get(entity_path)
        if stats is None:
            return
        stats.logged += 1
        stats.convert_sum_ms += convert_ms
        stats.convert_max_ms = max(stats.convert_max_ms, convert_ms)
        stats.log_sum_ms += log_ms
        stats.log_max_ms = max(stats.log_max_ms, log_ms)
        self._debug_maybe_log(entity_path, time.monotonic())

    def _debug_maybe_log(self, entity_path: str, now: float) -> None:
        stats = self._debug_stats.get(entity_path)
        if stats is None:
            return
        elapsed = now - stats.window_start
        if elapsed < self.config.debug_stats_interval:
            return

        recv_fps = stats.received / elapsed if elapsed > 0 else 0.0
        log_fps = stats.logged / elapsed if elapsed > 0 else 0.0
        frame_delta_avg_ms = (
            stats.frame_delta_sum_ms / stats.frame_delta_count if stats.frame_delta_count else 0.0
        )
        frame_delta_last_ms = (
            stats.frame_delta_last_ms if stats.frame_delta_last_ms is not None else 0.0
        )
        frame_delta_max_ms = (
            stats.frame_delta_max_ms if stats.frame_delta_max_ms is not None else 0.0
        )
        convert_avg_ms = stats.convert_sum_ms / stats.logged if stats.logged else 0.0
        log_avg_ms = stats.log_sum_ms / stats.logged if stats.logged else 0.0

        logger.info(
            "rerun recv stats "
            f"entity={entity_path} recv_fps={recv_fps:.2f} log_fps={log_fps:.2f} "
            f"received={stats.received} logged={stats.logged} throttled={stats.throttled} "
            f"recv_dt_last_ms={frame_delta_last_ms:.1f} "
            f"recv_dt_avg_ms={frame_delta_avg_ms:.1f} "
            f"recv_dt_max_ms={frame_delta_max_ms:.1f} "
            f"to_rerun_avg_ms={convert_avg_ms:.1f} to_rerun_max_ms={stats.convert_max_ms:.1f} "
            f"rr_log_avg_ms={log_avg_ms:.1f} rr_log_max_ms={stats.log_max_ms:.1f} "
            f"{stats.sample}"
        )
        low_fps_threshold = self._debug_low_fps_threshold(entity_path)
        if low_fps_threshold is not None and recv_fps < low_fps_threshold:
            logger.warning(
                "rerun low recv fps "
                f"entity={entity_path} recv_fps={recv_fps:.2f} "
                f"threshold_fps={low_fps_threshold:.2f} log_fps={log_fps:.2f} "
                f"received={stats.received} logged={stats.logged} throttled={stats.throttled} "
                f"recv_dt_last_ms={frame_delta_last_ms:.1f} "
                f"recv_dt_avg_ms={frame_delta_avg_ms:.1f} "
                f"recv_dt_max_ms={frame_delta_max_ms:.1f} "
                f"to_rerun_avg_ms={convert_avg_ms:.1f} to_rerun_max_ms={stats.convert_max_ms:.1f} "
                f"rr_log_avg_ms={log_avg_ms:.1f} rr_log_max_ms={stats.log_max_ms:.1f} "
                f"{stats.sample}"
            )
        self._debug_stats[entity_path] = _StreamDebugStats(window_start=now)

    def _on_message(self, msg: Any, topic: Any) -> None:
        """Handle incoming message - log to rerun."""

        entity_path: str = self._get_entity_path(topic)
        self._debug_note_received(entity_path, msg)

        # Throttle entities with a max_hz limit
        if entity_path in self._min_intervals:
            now = time.monotonic()
            if now - self._last_log.get(entity_path, 0.0) < self._min_intervals[entity_path]:
                self._debug_note_throttled(entity_path)
                return
            self._last_log[entity_path] = now

        convert_start = time.monotonic()
        rerun_data: RerunData | None = self._visual_override_for_entity_path(entity_path)(msg)
        convert_ms = (time.monotonic() - convert_start) * 1000.0

        if not rerun_data:
            self._debug_maybe_log(entity_path, time.monotonic())
            return

        if self.config.use_message_timestamps:
            # Place data on the timeline by capture time, not by when the bridge got
            # to it. This assumes the message timestamp uses a viewer-compatible clock.
            ts = getattr(msg, "ts", None)
            if ts is not None:
                rr.set_time("capture", timestamp=ts)

        log_start = time.monotonic()
        # TFMessage for example returns list of (entity_path, archetype) tuples
        if is_rerun_multi(rerun_data):
            for path, archetype in rerun_data:
                if self._latest_only_for_entity(path):
                    rr.log(path, rr.Clear(recursive=True))
                rr.log(path, archetype)
        else:
            if self._latest_only_for_entity(entity_path):
                rr.log(entity_path, rr.Clear(recursive=True))
            rr.log(entity_path, cast("Archetype", rerun_data))
            # if source msg carries a frame_id, attach the entity to that TF frame.
            # Skip Transform3D (it *is* the relation) and Pinhole (it carries its
            # own explicit parent_frame -- a second Transform3D would double-parent
            # the camera frame, which Rerun rejects).
            if not isinstance(rerun_data, (rr.Transform3D, rr.Pinhole)):
                frame_id = getattr(msg, "frame_id", None)
                if frame_id and self._frame_attached.get(entity_path) != frame_id:
                    rr.log(entity_path, rr.Transform3D(parent_frame=f"tf#/{frame_id}"))
                    self._frame_attached[entity_path] = frame_id
        log_ms = (time.monotonic() - log_start) * 1000.0
        self._debug_note_logged(entity_path, convert_ms, log_ms)

    @rpc
    def start(self) -> None:
        super().start()

        logger.info("Rerun bridge starting")

        self._last_log = {}
        self._frame_attached = {}
        self._debug_stats = {}
        self._debug_last_recv_time = {}
        self._min_intervals: dict[str, float] = {
            entity: 1.0 / hz for entity, hz in self.config.max_hz.items() if hz > 0
        }

        connect_url = self.config.connect_url
        if connect_url is None:
            connect_url = f"rerun+http://{self.host}:{RERUN_GRPC_PORT}/proxy"

        server_uri = rerun_init(
            start_grpc=True,
            grpc_config={
                "connect_url": connect_url,
                "server_memory_limit": self.config.memory_limit,
            },
        )
        assert server_uri is not None  # start_grpc=True guarantees a URI

        parsed = urlparse(connect_url.replace("rerun+", "", 1))
        grpc_port = parsed.port or RERUN_GRPC_PORT

        if self.config.rerun_open not in get_args(RerunOpenOption):
            logger.warning(
                f"rerun_open was {self.config.rerun_open} which is not one of "
                f"{get_args(RerunOpenOption)}"
            )

        spawned = False
        if self.config.rerun_open in ("native", "both"):
            try:
                import rerun_bindings

                # Use --connect so the viewer connects to the bridge's gRPC
                # server rather than starting its own (which would conflict).
                rerun_bindings.spawn(
                    executable_name="dimos-viewer",
                    memory_limit=self.config.memory_limit,
                    extra_args=["--connect", server_uri],
                )
                spawned = True
            except ImportError:
                pass  # dimos-viewer not installed
            except Exception:
                logger.warning(
                    "dimos-viewer found but failed to spawn, falling back to stock rerun",
                    exc_info=True,
                )

            # fallback on normal (non-dimos-viewer) rerun
            if not spawned:
                try:
                    rr.spawn(connect=True, memory_limit=self.config.memory_limit)
                    spawned = True
                except (RuntimeError, FileNotFoundError):
                    logger.warning(
                        "Rerun native viewer not available (headless?). "
                        "Bridge will continue without a viewer — data is still "
                        "accessible via --rerun-open web or by connecting a viewer to the gRPC server.",
                        exc_info=True,
                    )

        open_web = self.config.rerun_open == "web" or self.config.rerun_open == "both"
        if open_web or self.config.rerun_web:
            rr.serve_web_viewer(
                connect_to=server_uri,
                open_browser=open_web,
                web_port=self.config.web_port,
            )

        # TODO: `spawned` is supposed to be false when run on the G1 (because viewer doesn't have a display) somehow it returns true
        if (
            self.config.rerun_open == "none"
            or (self.config.rerun_open == "native" and not spawned)
            or self.host == "0.0.0.0"
        ):
            self._log_connect_hints(grpc_port)

        if self.config.blueprint:
            rr.send_blueprint(_with_graph_tab(self.config.blueprint()))

        # Resolve pubsubs lazily — the module-level global_config singleton in worker
        # processes doesn't have CLI overrides. Use self.config.g which is the parent's
        # updated config, passed via the worker kwargs.
        pubsubs = _resolve_pubsubs(self.config)

        # Start pubsubs and subscribe to all messages
        for pubsub in pubsubs:
            logger.info(f"bridge listening on {pubsub.__class__.__name__}")
            if hasattr(pubsub, "start"):
                pubsub.start()
            unsub = pubsub.subscribe_all(self._on_message)
            self.register_disposable(Disposable(unsub))

        # Add pubsub stop as disposable
        for pubsub in pubsubs:
            if hasattr(pubsub, "stop"):
                self.register_disposable(Disposable(pubsub.stop))  # type: ignore[union-attr]

        self._log_static()

    def _log_connect_hints(self, grpc_port: int) -> None:
        """Log CLI commands for connecting a viewer to this bridge."""
        local_ips = get_local_ips()
        local_grpc = f"rerun+http://{self.host}:{grpc_port}/proxy"
        local_ws = f"ws://{self.host}:{self.config.g.rerun_websocket_server_port}/ws"
        hostname = socket.gethostname()

        columns = 60
        lines = [
            "",
            "=" * columns,
            "Rerun gRPC server running (no viewer opened)",
            "",
            "Connect a viewer:",
            f"  dimos-viewer --connect {local_grpc} --ws-url {local_ws}",
        ]
        for ip, iface in local_ips:
            remote_grpc = f"rerun+http://{ip}:{grpc_port}/proxy"
            remote_ws = f"ws://{ip}:{self.config.g.rerun_websocket_server_port}/ws"
            lines.append(f"  dimos-viewer --connect {remote_grpc} --ws-url {remote_ws}  # {iface}")
        lines.append("")
        lines.append(f"  hostname: {hostname}")
        lines.append("=" * columns)
        lines.append("")

        logger.info("\n".join(lines))

    def _log_static(self) -> None:
        for entity_path, factory in self.config.static.items():
            data = factory(rr)
            if isinstance(data, list):
                for archetype in data:
                    rr.log(entity_path, archetype, static=True)
            else:
                rr.log(entity_path, data, static=True)

    @rpc
    def log_blueprint_graph(self, dot_code: str, module_names: list[str]) -> None:
        """Log a blueprint module graph from a Graphviz DOT string.

        Runs ``dot -Tplain`` to compute positions, then logs
        ``rr.GraphNodes`` + ``rr.GraphEdges`` to the active recording.

        Args:
            dot_code: The DOT-format graph (from ``introspection.blueprint.dot.render``).
            module_names: List of module class names (to distinguish modules from channels).
        """

        try:
            result = subprocess.run(
                ["dot", "-Tplain"], input=dot_code, text=True, capture_output=True, timeout=30
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return
        if result.returncode != 0:
            return

        node_ids: list[str] = []
        node_labels: list[str] = []
        node_colors: list[int] = []
        positions: list[tuple[float, float]] = []
        radii: list[float] = []
        edges: list[tuple[str, str]] = []
        module_set = set(module_names)

        for line in result.stdout.splitlines():
            if line.startswith("node "):
                parts = line.split()
                node_id = parts[1].strip('"')
                x = float(parts[2]) * self.GRAPH_VIZ_SCALE
                y = -float(parts[3]) * self.GRAPH_VIZ_SCALE
                label = parts[6].strip('"')
                color = parts[9].strip('"')

                node_ids.append(node_id)
                node_labels.append(label)
                positions.append((x, y))
                node_colors.append(_hex_to_rgba(color))
                radii.append(self.MODULE_RADIUS if node_id in module_set else self.CHANNEL_RADIUS)

            elif line.startswith("edge "):
                parts = line.split()
                edges.append((parts[1].strip('"'), parts[2].strip('"')))

        if not node_ids:
            return

        rr.log(
            "blueprint",
            rr.GraphNodes(
                node_ids=node_ids,
                labels=node_labels,
                colors=node_colors,
                positions=positions,
                radii=radii,
                show_labels=True,
            ),
            rr.GraphEdges(edges=edges, graph_type="directed"),
            static=True,
        )

    @rpc
    def stop(self) -> None:
        self._override_cache.clear()
        self._frame_attached.clear()
        super().stop()


def run_bridge(
    memory_limit: str = "25%",
    rerun_open: RerunOpenOption = RERUN_OPEN_DEFAULT,
    rerun_web: bool = RERUN_ENABLE_WEB,
) -> None:
    """Start a RerunBridgeModule with default LCM config and block until interrupted."""
    autoconf(check_only=True)

    bridge = RerunBridgeModule(
        memory_limit=memory_limit,
        rerun_open=rerun_open,
        rerun_web=rerun_web,
    )
    bridge.start()

    def _shutdown(*_: object) -> None:
        bridge.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.pause()
