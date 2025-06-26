### meta_writer.py
import json
from pathlib import Path
from typing import List, Dict
import cv2

class MetaWriter:
    """
    Handles writing info.json, tasks.jsonl, episodes.jsonl under meta/.
    """
    CODEBASE_VERSION = "v2.0"

    def __init__(
        self,
        root: Path,
        fps: int,
        chunk_size: int,
        task_name: str,
        total_frames: int,
        episodes: List[Dict],
    ):
        self.root = root
        self.fps = fps
        self.chunk_size = chunk_size
        self.task_name = task_name
        self.total_frames = total_frames
        self.episodes = episodes  # list of {"episode_index": int, "tasks": [task_name], "length": frame_count}

    def write_info(self) -> None:
        total_eps = len(self.episodes)
        total_chunks = (total_eps - 1) // self.chunk_size + 1 if total_eps > 0 else 0
        total_frames = sum(ep["length"] for ep in self.episodes)
        total_videos = total_eps * 2

        # === 自動偵測影片影像 shape ===
        sample_video_path = self.root / "videos/chunk-000/observation.images.image/episode_000000.mp4"
        if not sample_video_path.exists():
            raise FileNotFoundError(f"Sample video not found: {sample_video_path}")
        cap = cv2.VideoCapture(str(sample_video_path))
        ret, frame = cap.read()
        cap.release()
        if not ret:
            raise RuntimeError("Failed to read sample frame for shape detection.")
        height, width, channels = frame.shape
        info = {
            "codebase_version": self.CODEBASE_VERSION,
            "robot_type": "franka",
            "total_episodes": total_eps,
            "total_frames": total_frames,
            "total_tasks": 1,
            "total_videos": total_eps * 2,
            "total_chunks": total_chunks,
            "chunks_size": self.chunk_size,
            "fps": self.fps,
            "splits": {"train": f"0:{total_eps}"},
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": (
                "videos/chunk-{episode_chunk:03d}/{video_key}/"
                "episode_{episode_index:06d}.mp4"
            ),
            "features": {
                "observation.images.image_additional_view": {
                    "dtype": "video",
                    "shape": [channels, height, width],
                    "names": ["rgb", "height", "width"],
                    "info": {
                        "video.fps": float(self.fps),
                        "video.height": height,
                        "video.width": width,
                        "video.channels": channels,
                        "video.codec": "av1",
                        "video.pix_fmt": "yuv420p",
                        "video.is_depth_map": False,
                        "has_audio": False,
                    },
                },
                "observation.images.image": {
                    "dtype": "video",
                    "shape": [channels, height, width],
                    "names": ["rgb", "height", "width"],
                    "info": {
                        "video.fps": float(self.fps),
                        "video.height": height,
                        "video.width": width,
                        "video.channels": channels,
                        "video.codec": "av1",
                        "video.pix_fmt": "yuv420p",
                        "video.is_depth_map": False,
                        "has_audio": False,
                    },
                },
                "observation.state": {
                    "dtype": "float32",
                    "shape": [7],
                    "names": {"motors": ["x", "y", "z", "roll", "pitch", "yaw", "width"]},
                },
                "action": {
                    "dtype": "float32",
                    "shape": [7],
                    "names": {"motors": ["x", "y", "z", "roll", "pitch", "yaw", "width"]},
                },
                "timestamp": {"dtype": "float32", "shape": [1], "names": None},
                "frame_index": {"dtype": "int64", "shape": [1], "names": None},
                "episode_index": {"dtype": "int64", "shape": [1], "names": None},
                "index": {"dtype": "int64", "shape": [1], "names": None},
                "task_index": {"dtype": "int64", "shape": [1], "names": None},
            },
        }
        with open(self.root / "meta" / "info.json", "w") as f:
            json.dump(info, f, indent=4)

    def write_tasks(self) -> None:
        task_file = self.root / "meta" / "tasks.jsonl"
        with open(task_file, "w") as f:
            f.write(json.dumps({"task_index": 0, "task": self.task_name}) + "\n")

    def write_episodes(self) -> None:
        ep_file = self.root / "meta" / "episodes.jsonl"
        with open(ep_file, "w") as f:
            for ep in self.episodes:
                f.write(json.dumps(ep) + "\n")

    def write_all(self) -> None:
        print("[MetaWriter] write_all() called")
        meta_dir = self.root / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True) 
        self.write_info()
        self.write_tasks()
        self.write_episodes()