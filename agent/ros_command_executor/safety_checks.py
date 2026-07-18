#!/usr/bin/env python3

from dataclasses import dataclass

import yaml


@dataclass
class SafetyCheckResult:
    name: str
    ok: bool
    detail: str
    value: object = None

    def as_dict(self):
        result = {
            "name": self.name,
            "ok": self.ok,
            "detail": self.detail,
        }
        if self.value is not None:
            result["value"] = self.value
        return result


class RosSafetyChecker:
    def __init__(self, run_command, ros_master_reachable):
        self._run = run_command
        self._ros_master_reachable = ros_master_reachable

    def run_checks(self, config):
        checks_config = config.get("checks") or {}
        results = []
        missing_topics = set()
        no_publisher_topics = set()
        no_subscriber_topics = set()

        if checks_config.get("require_ros_master", True):
            master_ok = bool(self._ros_master_reachable())
            results.append(
                SafetyCheckResult(
                    name="ros_master",
                    ok=master_ok,
                    detail="ROS master is reachable" if master_ok else "ROS master is not reachable",
                )
            )
            if not master_ok:
                return results

        nodes = None
        topics = None

        required_nodes = checks_config.get("required_nodes") or []
        if required_nodes:
            nodes = self._safe_list_nodes(checks_config, results)
            if nodes is not None:
                missing = [node for node in required_nodes if node not in nodes]
                results.append(
                    SafetyCheckResult(
                        name="required_nodes",
                        ok=not missing,
                        detail="required nodes are present"
                        if not missing
                        else "missing required nodes: %s" % ", ".join(missing),
                        value={"required": required_nodes, "missing": missing},
                    )
                )

        required_topics = checks_config.get("required_topics") or []
        if required_topics:
            topics = self._safe_list_topics(checks_config, results)
            if topics is not None:
                missing = [topic for topic in required_topics if topic not in topics]
                missing_topics = set(missing)
                results.append(
                    SafetyCheckResult(
                        name="required_topics",
                        ok=not missing,
                        detail="required topics are present"
                        if not missing
                        else "missing required topics: %s" % ", ".join(missing),
                        value={"required": required_topics, "missing": missing},
                    )
                )
                for topic in required_topics:
                    if topic not in missing_topics and not self._topic_has_publisher(topic, checks_config):
                        no_publisher_topics.add(topic)
                if no_publisher_topics:
                    results.append(
                        SafetyCheckResult(
                            name="topic_publishers",
                            ok=False,
                            detail="topics have no publishers: %s"
                            % ", ".join(sorted(no_publisher_topics)),
                            value={"topics": sorted(no_publisher_topics)},
                    )
                )

        required_subscriber_topics = checks_config.get("required_subscriber_topics") or []
        if required_subscriber_topics:
            if topics is None:
                topics = self._safe_list_topics(checks_config, results)
            if topics is not None:
                missing = [topic for topic in required_subscriber_topics if topic not in topics]
                results.append(
                    SafetyCheckResult(
                        name="required_subscriber_topics",
                        ok=not missing,
                        detail="required subscriber topics are present"
                        if not missing
                        else "missing required subscriber topics: %s" % ", ".join(missing),
                        value={"required": required_subscriber_topics, "missing": missing},
                    )
                )
                for topic in required_subscriber_topics:
                    if topic not in missing and not self._topic_has_subscriber(topic, checks_config):
                        no_subscriber_topics.add(topic)
                if no_subscriber_topics:
                    results.append(
                        SafetyCheckResult(
                            name="topic_subscribers",
                            ok=False,
                            detail="topics have no subscribers: %s"
                            % ", ".join(sorted(no_subscriber_topics)),
                            value={"topics": sorted(no_subscriber_topics)},
                        )
                    )

        state_check = checks_config.get("state")
        if state_check:
            results.extend(
                self._check_state(state_check, checks_config, missing_topics, no_publisher_topics)
            )

        extended_state_check = checks_config.get("extended_state")
        if extended_state_check:
            results.extend(
                self._check_extended_state(
                    extended_state_check,
                    checks_config,
                    missing_topics,
                    no_publisher_topics,
                )
            )

        battery_check = checks_config.get("battery")
        if battery_check:
            results.extend(
                self._check_battery(battery_check, checks_config, missing_topics, no_publisher_topics)
            )

        for sample in checks_config.get("sample_topics") or []:
            results.append(
                self._check_topic_sample(sample, checks_config, missing_topics, no_publisher_topics)
            )

        return results

    def _safe_list_nodes(self, checks_config, results):
        try:
            output = self._run(["rosnode", "list"], timeout=checks_config.get("list_timeout", 2.0))
            return [line.strip() for line in output.splitlines() if line.strip()]
        except Exception as exc:
            results.append(
                SafetyCheckResult(
                    name="rosnode_list",
                    ok=False,
                    detail="failed to list ROS nodes: %s" % exc,
                )
            )
            return None

    def _safe_list_topics(self, checks_config, results):
        try:
            output = self._run(["rostopic", "list"], timeout=checks_config.get("list_timeout", 2.0))
            return [line.strip() for line in output.splitlines() if line.strip()]
        except Exception as exc:
            results.append(
                SafetyCheckResult(
                    name="rostopic_list",
                    ok=False,
                    detail="failed to list ROS topics: %s" % exc,
                )
            )
            return None

    def _topic_has_publisher(self, topic, checks_config):
        try:
            output = self._run(
                ["rostopic", "info", topic],
                timeout=checks_config.get("info_timeout", 1.0),
            )
        except Exception:
            return False
        return "Publishers: None" not in output

    def _topic_has_subscriber(self, topic, checks_config):
        try:
            output = self._run(
                ["rostopic", "info", topic],
                timeout=checks_config.get("info_timeout", 1.0),
            )
        except Exception:
            return False
        return "Subscribers: None" not in output

    def _check_state(self, state_check, checks_config, missing_topics, no_publisher_topics):
        name = state_check.get("name", "mavros_state")
        topic = state_check["topic"]
        if topic in missing_topics:
            return [
                SafetyCheckResult(
                    name=name,
                    ok=False,
                    detail="topic is missing: %s" % topic,
                )
            ]
        if topic in no_publisher_topics:
            return [
                SafetyCheckResult(
                    name=name,
                    ok=False,
                    detail="topic has no publisher: %s" % topic,
                )
            ]
        message, error = self._sample_topic(topic, checks_config)
        if error:
            return [SafetyCheckResult(name=name, ok=False, detail=error)]

        results = [
            SafetyCheckResult(
                name=name,
                ok=True,
                detail="received message from %s" % topic,
            )
        ]

        if state_check.get("require_connected", False):
            connected = bool(message.get("connected", False))
            results.append(
                SafetyCheckResult(
                    name="%s.connected" % name,
                    ok=connected,
                    detail="PX4 is connected" if connected else "PX4 is not connected",
                    value=connected,
                )
            )

        if "require_armed" in state_check:
            expected = bool(state_check["require_armed"])
            armed = bool(message.get("armed", False))
            results.append(
                SafetyCheckResult(
                    name="%s.armed" % name,
                    ok=armed == expected,
                    detail="armed=%s" % armed,
                    value=armed,
                )
            )

        allowed_modes = state_check.get("allowed_modes")
        if allowed_modes:
            mode = str(message.get("mode", ""))
            results.append(
                SafetyCheckResult(
                    name="%s.mode" % name,
                    ok=mode in [str(item) for item in allowed_modes],
                    detail="mode=%s" % mode,
                    value=mode,
                )
            )

        return results

    def _check_extended_state(
        self,
        extended_state_check,
        checks_config,
        missing_topics,
        no_publisher_topics,
    ):
        name = extended_state_check.get("name", "mavros_extended_state")
        topic = extended_state_check["topic"]
        if topic in missing_topics:
            return [
                SafetyCheckResult(
                    name=name,
                    ok=False,
                    detail="topic is missing: %s" % topic,
                )
            ]
        if topic in no_publisher_topics:
            return [
                SafetyCheckResult(
                    name=name,
                    ok=False,
                    detail="topic has no publisher: %s" % topic,
                )
            ]
        message, error = self._sample_topic(topic, checks_config)
        if error:
            return [SafetyCheckResult(name=name, ok=False, detail=error)]

        results = [
            SafetyCheckResult(
                name=name,
                ok=True,
                detail="received message from %s" % topic,
            )
        ]

        allowed_landed_states = extended_state_check.get("allowed_landed_states")
        if allowed_landed_states is not None:
            landed_state = message.get("landed_state")
            results.append(
                SafetyCheckResult(
                    name="%s.landed_state" % name,
                    ok=landed_state in allowed_landed_states,
                    detail="landed_state=%s" % landed_state,
                    value=landed_state,
                )
            )

        return results

    def _check_battery(self, battery_check, checks_config, missing_topics, no_publisher_topics):
        name = battery_check.get("name", "battery")
        topic = battery_check["topic"]
        if topic in missing_topics:
            return [
                SafetyCheckResult(
                    name=name,
                    ok=False,
                    detail="topic is missing: %s" % topic,
                )
            ]
        if topic in no_publisher_topics:
            return [
                SafetyCheckResult(
                    name=name,
                    ok=False,
                    detail="topic has no publisher: %s" % topic,
                )
            ]
        message, error = self._sample_topic(topic, checks_config)
        if error:
            return [SafetyCheckResult(name=name, ok=False, detail=error)]

        results = [
            SafetyCheckResult(
                name=name,
                ok=True,
                detail="received message from %s" % topic,
            )
        ]

        min_percentage = battery_check.get("min_percentage")
        if min_percentage is not None:
            percentage = message.get("percentage")
            ok = percentage is not None and float(percentage) >= float(min_percentage)
            results.append(
                SafetyCheckResult(
                    name="%s.percentage" % name,
                    ok=ok,
                    detail="percentage=%s, min=%s" % (percentage, min_percentage),
                    value=percentage,
                )
            )

        min_voltage = battery_check.get("min_voltage")
        if min_voltage is not None:
            voltage = message.get("voltage")
            ok = voltage is not None and float(voltage) >= float(min_voltage)
            results.append(
                SafetyCheckResult(
                    name="%s.voltage" % name,
                    ok=ok,
                    detail="voltage=%s, min=%s" % (voltage, min_voltage),
                    value=voltage,
                )
            )

        return results

    def _check_topic_sample(self, sample, checks_config, missing_topics, no_publisher_topics):
        if isinstance(sample, str):
            sample = {"topic": sample}
        name = sample.get("name", "topic_sample:%s" % sample["topic"])
        topic = sample["topic"]
        if topic in missing_topics:
            return SafetyCheckResult(
                name=name,
                ok=False,
                detail="topic is missing: %s" % topic,
            )
        if topic in no_publisher_topics:
            return SafetyCheckResult(
                name=name,
                ok=False,
                detail="topic has no publisher: %s" % topic,
            )
        _message, error = self._sample_topic(topic, checks_config)
        return SafetyCheckResult(
            name=name,
            ok=error is None,
            detail="received message from %s" % topic if error is None else error,
        )

    def _sample_topic(self, topic, checks_config):
        try:
            output = self._run(
                ["rostopic", "echo", "-n", "1", topic],
                timeout=checks_config.get("sample_timeout", 2.0),
            )
        except Exception as exc:
            return None, "failed to receive message from %s: %s" % (topic, exc)

        try:
            docs = [doc for doc in yaml.safe_load_all(output) if doc is not None]
        except yaml.YAMLError as exc:
            return None, "failed to parse message from %s: %s" % (topic, exc)

        if not docs:
            return None, "received empty message from %s" % topic

        message = docs[0]
        if not isinstance(message, dict):
            return None, "message from %s is not a mapping" % topic
        return message, None
