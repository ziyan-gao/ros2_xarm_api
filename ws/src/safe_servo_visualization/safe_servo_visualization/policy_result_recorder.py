"""Atomic, timestamped policy experiment results; independent of ROS."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
from uuid import uuid4


def timestamp():
    return datetime.now(timezone.utc).isoformat()


class PolicyResultRecorder:
    def __init__(self, directory, config):
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / (
            datetime.now(timezone.utc).strftime('policy_%Y%m%dT%H%M%S_%fZ_')
            + uuid4().hex[:8] + '.json')
        self.lock = threading.RLock()
        self.data = dict(schema_version=1, started_at=timestamp(), config=config,
                         utilization_basis='virtual_volume_including_clearance',
                         events=[], operations=[], summary={})
        self.data['utilization_bases'] = {
            'utilization_measured': 'unrounded_measured_volume_of_packed_items',
            'utilization_planning': 'rounded_dimension_volume_without_clearance',
            'utilization_including_clearance': 'virtual_volume_including_clearance',
        }
        self._save()

    def append(self, kind, details, summary):
        with self.lock:
            event = dict(timestamp=timestamp(), event=kind, **details)
            self.data['events'].append(event)
            if kind == 'operation_completed':
                self.data['operations'].append(event)
            self.data['summary'] = summary
            self._save()

    def _save(self):
        self.data['updated_at'] = timestamp()
        temporary = self.path.with_suffix('.json.tmp')
        with temporary.open('w', encoding='utf-8') as stream:
            json.dump(self.data, stream, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.path)
