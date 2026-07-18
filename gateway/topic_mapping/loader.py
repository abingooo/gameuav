#!/usr/bin/env python3

import os

import yaml


class TopicMappingError(ValueError):
    pass


def load_topic_mapping(path):
    if not os.path.exists(path):
        raise TopicMappingError("topic mapping file not found: %s" % path)
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if "gateway_mapping" not in data:
        raise TopicMappingError("mapping must contain gateway_mapping")
    return data


def get_gateway_mapping(data):
    return data.get("gateway_mapping") or {}


def get_inbound_mapping(data):
    return get_gateway_mapping(data).get("inbound") or {}


def get_outbound_mapping(data):
    return get_gateway_mapping(data).get("outbound") or {}
