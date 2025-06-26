### video_writer.py
'''
video needs to convert to libx264 decoede to visualize with html
'''
import cv2
import tempfile
import subprocess
from pathlib import Path
import numpy as np

class VideoWriterModule:
    """
    Handles saving a sequence of frames (np.ndarray) to an MP4 file with H.264 codec.
    """
    def __init__(self, fps: int):
        self.fps = fps

    def write_episode(self, frames: np.ndarray, output_path: Path) -> None:
        """
        Save frames to output_path as H.264 encoded MP4.
        frames: np.ndarray of shape (N, H, W, 3), dtype=uint8
        """
        if frames.size == 0:
            return
        height, width = frames.shape[1], frames.shape[2]

        with tempfile.NamedTemporaryFile(suffix=".avi", delete=False) as tmpfile:
            avi_path = tmpfile.name

        # Step 1: Save as .avi using OpenCV
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        writer = cv2.VideoWriter(avi_path, fourcc, self.fps, (width, height))
        if not writer.isOpened():
            raise RuntimeError(f"Cannot open temp avi writer for {avi_path}")
        for frame in frames:
            writer.write(frame)
        writer.release()

        # Step 2: Use ffmpeg to re-encode as .mp4 (H.264)
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            'ffmpeg',
            '-y',  # Overwrite
            '-i', avi_path,
            '-vcodec', 'libx264',
            '-pix_fmt', 'yuv420p',
            str(output_path)
        ]
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.returncode != 0:
            print(result.stderr.decode())
            raise RuntimeError("FFmpeg failed to convert video.")
        
        Path(avi_path).unlink()  # Clean up
