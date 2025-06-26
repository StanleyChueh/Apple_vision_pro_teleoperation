### parquet_writer.py
import numpy as np
import pandas as pd
from pathlib import Path

class ParquetWriter:
    """
    Handles saving a single episode to a Parquet file with the following schema:
      - observation.state: list of length 7 ([x, y, z, roll, pitch, yaw, gripper])
      - action: list of length 7 (difference between successive states)
      - timestamp, frame_index, episode_index, index, task_index
    """
    def __init__(self, fps: int):
        self.fps = fps

    def write_episode(
        self,
        episode_index: int,
        eef_pos: np.ndarray,
        eef_ori: np.ndarray,
        gripper: np.ndarray,
        output_path: Path,
    ) -> None:
        frame_count = len(gripper)
        # Build state array: shape (frame_count, 7)
        state_array = np.concatenate(
            [eef_pos, eef_ori, gripper.reshape(-1, 1)], axis=1
        )
        # Compute action: (t+1 state - t state), last frame zero
        action_diff = state_array[1:] - state_array[:-1]
        last_zero = np.zeros((1, state_array.shape[1]), dtype=state_array.dtype)
        action_array = np.vstack([action_diff, last_zero])
        # Timestamps and indices
        timestamps = np.arange(frame_count) / self.fps
        frame_idx = np.arange(frame_count, dtype=int)
        ep_idx = np.full(frame_count, episode_index, dtype=int)
        absolute_index = episode_index * frame_count + frame_idx
        task_idx = np.zeros(frame_count, dtype=int)
        # Convert to list-of-lists for Parquet
        obs_state_list = [state_array[i].tolist() for i in range(frame_count)]
        action_list = [action_array[i].tolist() for i in range(frame_count)]
        # Build DataFrame
        df = pd.DataFrame({
            "observation.state": obs_state_list,
            "action": action_list,
            "timestamp": timestamps,
            "frame_index": frame_idx,
            "episode_index": ep_idx,
            "index": absolute_index,
            "task_index": task_idx,
        })
        # Write to Parquet
        df.to_parquet(output_path, index=False)