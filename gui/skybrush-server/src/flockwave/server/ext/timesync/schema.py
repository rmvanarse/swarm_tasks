"""Configuration schema for the `timesync` extension."""

from .constants import (
    DEFAULT_OFFSET_LOG_THRESHOLD,
    DEFAULT_SOURCE_EXPIRY_THRESHOLD,
    DEFAULT_SYNC_THRESHOLD,
)

__all__ = ("schema",)


schema = {
    "properties": {
        "sync_threshold": {
            "type": "number",
            "title": "Sync threshold",
            "description": (
                "Maximum absolute clock offset, in seconds, for the local server "
                "clock to be considered synchronized to wall clock time."
            ),
            "default": DEFAULT_SYNC_THRESHOLD,
            "minimum": 0,
        },
        "source_expiry_threshold": {
            "type": "number",
            "title": "Source expiry threshold",
            "description": (
                "Maximum age of a timestamp submission, in seconds, after which "
                "it is ignored."
            ),
            "default": DEFAULT_SOURCE_EXPIRY_THRESHOLD,
            "exclusiveMinimum": 0,
        },
        "offset_log_threshold": {
            "type": "number",
            "title": "Offset log threshold",
            "description": (
                "Minimum change in the selected clock offset, in seconds, that "
                "triggers a new log message while the synchronization state stays the same."
            ),
            "default": DEFAULT_OFFSET_LOG_THRESHOLD,
            "minimum": 0,
        },
    }
}
