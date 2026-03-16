from __future__ import annotations

from std_msgs.msg import String


class EventEmitterMixin:
    """Mixin for ROS2 nodes that publish lifecycle/debug event strings.

    Call ``_setup_event_emitter(event_topic, publish_events)`` in ``__init__``
    after the node's publisher infrastructure is ready, then use
    ``emit_event(name)`` anywhere in the node.
    """

    def _setup_event_emitter(self, event_topic: str, publish_events: bool) -> None:
        self.publish_events: bool = publish_events
        self.events_pub = (
            self.create_publisher(String, event_topic, 10)
            if publish_events
            else None
        )

    def emit_event(self, name: str) -> None:
        if self.events_pub is None:
            return
        msg = String()
        msg.data = str(name)
        self.events_pub.publish(msg)
