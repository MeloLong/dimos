use std::future::Future;
use std::io;
use std::sync::Arc;

/// Per-channel dispatch closure handed to a transport on `subscribe`. The
/// transport calls it with each message's raw payload. Decode and routing
/// happen inside it.
pub type Dispatch = Arc<dyn Fn(&[u8]) + Send + Sync>;

/// Abstraction over the message transport used by a native module.
///
/// New transport protocols should implement this trait.
/// `NativeModule` is generic over any transport
pub trait Transport: Send + Sync + 'static {
    /// Send `data` on `channel`.
    fn publish(&self, channel: &str, data: Vec<u8>) -> impl Future<Output = io::Result<()>> + Send;
    /// Deliver each message on `channel` to `on_msg`.
    fn subscribe(
        &self,
        channel: &str,
        on_msg: Dispatch,
    ) -> impl Future<Output = io::Result<()>> + Send;

    /// Apply the per-channel publisher QoS the coordinator sends. The value is
    /// the `qos` object from the stdin config, or null when absent. Transports
    /// without per-topic QoS ignore it.
    fn set_publisher_qos(&self, _qos: &serde_json::Value) {}
}
