"""JSON persistence for portable, inspectable VLA episodes."""

from __future__ import annotations

import json
from pathlib import Path

from .contracts import Episode


def save_episode(episode: Episode, path: str | Path) -> Path:
    """Validate through the dataclass and atomically write readable JSON."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(
        json.dumps(episode.to_dict(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)
    return destination


def load_episode(path: str | Path) -> Episode:
    """Read an episode and reject data that no longer matches the contract."""

    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("episode JSON root must be an object")
    return Episode.from_dict(payload)
