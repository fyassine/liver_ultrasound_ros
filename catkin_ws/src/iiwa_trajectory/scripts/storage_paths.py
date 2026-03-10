#!/usr/bin/env python3

import os


RECORDINGS_DIR = os.path.normpath(
    os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            '..',
            '..',
            '..',
            '..',
            'data',
        )
    )
)


def ensure_recordings_dir():
    os.makedirs(RECORDINGS_DIR, exist_ok=True)
    return RECORDINGS_DIR


def default_bag_path(filename):
    return os.path.join(RECORDINGS_DIR, filename)


def resolve_bag_path(path_value):
    if not path_value:
        return path_value
    expanded = os.path.abspath(os.path.expanduser(path_value))
    if os.path.isabs(path_value) or os.path.exists(expanded):
        return expanded
    return default_bag_path(path_value)