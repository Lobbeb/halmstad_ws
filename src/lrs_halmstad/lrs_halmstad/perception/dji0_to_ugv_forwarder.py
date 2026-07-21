from __future__ import annotations

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from lrs_halmstad_interfaces.msg import AerialHazardArray
from lrs_halmstad.perception.detection_protocol import decode_detection_payload, encode_detection_payload
from lrs_halmstad.perception.detection_status import parse_status_line


def _coerce_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class Dji0ToUgvForwarder(Node):
    def __init__(self) -> None:
        super().__init__("dji0_to_ugv_forwarder")

        self.in_detection_topic = str(
            self.declare_parameter("in_detection_topic", "/coord/dji0/leader_detection").value
        ).strip() or "/coord/dji0/leader_detection"
        self.in_status_topic = str(
            self.declare_parameter("in_status_topic", "/coord/dji0/leader_detection_status").value
        ).strip() or "/coord/dji0/leader_detection_status"

        self.out_detection_topic = str(
            self.declare_parameter("out_detection_topic", "/coord/ugv/leader_detection").value
        ).strip() or "/coord/ugv/leader_detection"
        self.out_status_topic = str(
            self.declare_parameter("out_status_topic", "/coord/ugv/leader_detection_status").value
        ).strip() or "/coord/ugv/leader_detection_status"
        self.in_summary_topic = str(
            self.declare_parameter("in_summary_topic", "/coord/dji0/support_observation_summary").value
        ).strip() or "/coord/dji0/support_observation_summary"
        self.out_summary_topic = str(
            self.declare_parameter("out_summary_topic", "/coord/ugv/support_observation_summary").value
        ).strip() or "/coord/ugv/support_observation_summary"
        self.awareness_enable = _coerce_bool(self.declare_parameter("awareness_enable", True).value)
        self.out_awareness_status_topic = str(
            self.declare_parameter("out_awareness_status_topic", "/coord/ugv/support_awareness_status").value
        ).strip() or "/coord/ugv/support_awareness_status"
        self.publish_advisory = _coerce_bool(self.declare_parameter("publish_advisory", True).value)
        self.out_advisory_topic = str(
            self.declare_parameter("out_advisory_topic", "/coord/ugv/support_path_advisory").value
        ).strip() or "/coord/ugv/support_path_advisory"

        self.forward_owner = str(self.declare_parameter("forward_owner", "dji0").value).strip() or "dji0"
        self.forward_stage = str(self.declare_parameter("forward_stage", "dji0_to_ugv").value).strip() or "dji0_to_ugv"
        self.hazard_forward_enable = _coerce_bool(
            self.declare_parameter("hazard_forward_enable", False).value
        )
        self.in_hazard_topic = str(
            self.declare_parameter("in_hazard_topic", "/coord/dji0/aerial_hazards").value
        ).strip() or "/coord/dji0/aerial_hazards"
        self.out_hazard_topic = str(
            self.declare_parameter("out_hazard_topic", "/coord/ugv/aerial_hazards").value
        ).strip() or "/coord/ugv/aerial_hazards"

        self._out_detection_pub = self.create_publisher(String, self.out_detection_topic, 10)
        self._out_status_pub = self.create_publisher(String, self.out_status_topic, 10)
        self._out_summary_pub = self.create_publisher(String, self.out_summary_topic, 10)
        self._out_awareness_pub = (
            self.create_publisher(String, self.out_awareness_status_topic, 10)
            if self.awareness_enable
            else None
        )
        self._out_advisory_pub = (
            self.create_publisher(String, self.out_advisory_topic, 10)
            if self.publish_advisory
            else None
        )
        self._out_hazard_pub = (
            self.create_publisher(AerialHazardArray, self.out_hazard_topic, 5)
            if self.hazard_forward_enable
            else None
        )

        self.create_subscription(String, self.in_detection_topic, self._on_detection, 10)
        self.create_subscription(String, self.in_status_topic, self._on_status, 10)
        self.create_subscription(String, self.in_summary_topic, self._on_summary, 10)
        if self.hazard_forward_enable:
            self.create_subscription(
                AerialHazardArray,
                self.in_hazard_topic,
                self._on_hazard,
                5,
            )

        self.get_logger().info(
            "[dji0_to_ugv_forwarder] Started: "
            f"in_detection={self.in_detection_topic}, in_status={self.in_status_topic}, "
            f"out_detection={self.out_detection_topic}, out_status={self.out_status_topic}, "
            f"in_summary={self.in_summary_topic}, out_summary={self.out_summary_topic}, "
            f"awareness_enable={self.awareness_enable}, awareness_status={self.out_awareness_status_topic}, "
            f"publish_advisory={self.publish_advisory}, advisory={self.out_advisory_topic}, "
            f"forward_owner={self.forward_owner}, forward_stage={self.forward_stage}, "
            f"hazard_forward_enable={self.hazard_forward_enable}, "
            f"hazard_in={self.in_hazard_topic}, hazard_out={self.out_hazard_topic}"
        )

    def _on_hazard(self, msg: AerialHazardArray) -> None:
        if self._out_hazard_pub is not None:
            self._out_hazard_pub.publish(msg)

    def _on_detection(self, msg: String) -> None:
        out = String()
        now_ns = int(self.get_clock().now().nanoseconds)

        try:
            det_msg = decode_detection_payload(msg.data)
            metadata = dict(det_msg.metadata or {})
            metadata["forward_stage"] = self.forward_stage
            metadata["forward_owner"] = self.forward_owner
            metadata["forwarded_from_topic"] = self.in_detection_topic
            metadata["forwarded_ros_ns"] = str(now_ns)
            out.data = encode_detection_payload(det_msg.stamp_ns or now_ns, det_msg.detection, metadata=metadata)
        except Exception:
            # Keep forwarding resilient even if upstream payload changes unexpectedly.
            out.data = msg.data

        self._out_detection_pub.publish(out)

    def _on_status(self, msg: String) -> None:
        base = str(msg.data).strip()
        suffix = f" forward_stage={self.forward_stage} forward_owner={self.forward_owner}"
        if not base:
            out_line = f"task=forward state=NO_INPUT reason=empty_upstream_status{suffix}"
        else:
            fields = parse_status_line(base)
            if (
                fields.get("forward_stage", "").strip() == self.forward_stage
                and fields.get("forward_owner", "").strip() == self.forward_owner
            ):
                out_line = base
            else:
                out_line = f"{base}{suffix}"

        out = String()
        out.data = out_line
        self._out_status_pub.publish(out)

    def _on_summary(self, msg: String) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        out = String()
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("summary_payload_not_object")
            payload["forward_stage"] = self.forward_stage
            payload["forward_owner"] = self.forward_owner
            payload["forwarded_from_topic"] = self.in_summary_topic
            payload["forwarded_ros_ns"] = now_ns
            out.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
            self._publish_awareness_status(payload)
            self._publish_advisory(payload)
        except Exception:
            out.data = msg.data
            self._publish_awareness_status(None, reason="invalid_summary_json")
            self._publish_advisory(None, reason="invalid_summary_json")
        self._out_summary_pub.publish(out)

    def _publish_awareness_status(self, payload: dict | None, *, reason: str = "") -> None:
        if self._out_awareness_pub is None:
            return

        if not isinstance(payload, dict):
            line = (
                "task=ugv_support_awareness state=INVALID_SUMMARY "
                f"reason={reason or 'invalid_summary'} selected_source=none "
                f"relation_source=unknown relation_quality=unknown possible_candidate=false "
                f"forward_stage={self.forward_stage} forward_owner={self.forward_owner} replanning=false"
            )
        else:
            selected_source = str(payload.get("selected_source", "none") or "none")
            sources = payload.get("sources", [])
            selected = None
            if isinstance(sources, list):
                selected = next(
                    (
                        source
                        for source in sources
                        if isinstance(source, dict)
                        and str(source.get("source_id", "")) == selected_source
                    ),
                    None,
                )
            if selected is None and isinstance(sources, list):
                selected = next((source for source in sources if isinstance(source, dict)), {})
            if not isinstance(selected, dict):
                selected = {}

            state = str(payload.get("state", "UNKNOWN") or "UNKNOWN")
            summary_reason = str(payload.get("reason", "none") or "none")
            selected_state = str(selected.get("state", state) or state)
            selected_reason = str(selected.get("reason", summary_reason) or summary_reason)
            conf = float(selected.get("conf", -1.0) or -1.0)
            cls_name = str(selected.get("cls_name", "") or "")
            cls_id = selected.get("cls_id", None)
            age_ms = float(selected.get("age_ms", -1.0) or -1.0)
            fresh = bool(selected.get("fresh", False))
            valid = bool(selected.get("valid", False))
            possible_candidate = valid and state == "OK" and conf >= 0.0
            line = (
                "task=ugv_support_awareness "
                f"state={state} reason={summary_reason} selected_source={selected_source} "
                f"selected_state={selected_state} selected_reason={selected_reason} "
                f"conf={conf:.3f} cls_id={cls_id if cls_id is not None else 'none'} "
                f"cls_name={cls_name or 'none'} source_age_ms={age_ms:.1f} "
                f"source_fresh={str(fresh).lower()} possible_candidate={str(possible_candidate).lower()} "
                f"relation_source={payload.get('relation_source', 'unknown')} "
                f"relation_quality={payload.get('relation_quality', 'unknown')} "
                f"forward_stage={self.forward_stage} forward_owner={self.forward_owner} "
                "replanning=false"
            )

        msg = String()
        msg.data = line
        self._out_awareness_pub.publish(msg)

    def _publish_advisory(self, payload: dict | None, *, reason: str = "") -> None:
        if self._out_advisory_pub is None:
            return

        if not isinstance(payload, dict):
            line = (
                "task=ugv_support_path_advisory advisory_state=INVALID_SUMMARY "
                f"reason={reason or 'invalid_summary'} candidate_available=false selected_source=none "
                "candidate_class=none confidence=-1.000 freshness_age_ms=-1.0 relation_source=unknown "
                "relation_quality=unknown suggested_action=monitor_only replanning_enabled=false"
            )
        else:
            selected_source = str(payload.get("selected_source", "none") or "none")
            sources = payload.get("sources", [])
            selected = None
            if isinstance(sources, list):
                selected = next(
                    (
                        source
                        for source in sources
                        if isinstance(source, dict)
                        and str(source.get("source_id", "")) == selected_source
                    ),
                    None,
                )
            if selected is None and isinstance(sources, list):
                selected = next((source for source in sources if isinstance(source, dict)), {})
            if not isinstance(selected, dict):
                selected = {}

            state = str(payload.get("state", "UNKNOWN") or "UNKNOWN")
            valid = bool(selected.get("valid", False))
            conf = float(selected.get("conf", -1.0) or -1.0)
            candidate_available = valid and state == "OK" and conf >= 0.0
            advisory_state = "CANDIDATE" if candidate_available else state
            cls_name = str(selected.get("cls_name", "") or "")
            cls_id = selected.get("cls_id", None)
            candidate_class = cls_name or (str(cls_id) if cls_id is not None else "none")
            age_ms = float(selected.get("age_ms", -1.0) or -1.0)
            line = (
                "task=ugv_support_path_advisory "
                f"advisory_state={advisory_state} candidate_available={str(candidate_available).lower()} "
                f"selected_source={selected_source} candidate_class={candidate_class} "
                f"confidence={conf:.3f} freshness_age_ms={age_ms:.1f} "
                f"relation_source={payload.get('relation_source', 'unknown')} "
                f"relation_quality={payload.get('relation_quality', 'unknown')} "
                "suggested_action=monitor_only replanning_enabled=false"
            )

        msg = String()
        msg.data = line
        self._out_advisory_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Dji0ToUgvForwarder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
