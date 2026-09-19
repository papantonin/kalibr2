#!/usr/bin/env python3
"""Run installed Kalibr with read-only observation accounting (ROS 1 environment)."""
import argparse
import json
import os
from pathlib import Path
import random
import runpy
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--counts", required=True)
parser.add_argument("--kalibr-script", required=True)
args, remaining = parser.parse_known_args()
if remaining[:1] == ["--"]:
    remaining = remaining[1:]
sys.path.insert(0, str(Path(args.kalibr_script).parent))
import numpy as np
import kalibr_common as kc

random.seed(0)
np.random.seed(0)
counts = {"extractions": [], "camera_error_terms": []}

def save():
    path = Path(args.counts)
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(counts, indent=2) + "\n")
    os.replace(str(temp), str(path))

original_extract = kc.extractCornersFromDataset

def extract(dataset, *positional, **keywords):
    start = time.monotonic()
    observations = original_extract(dataset, *positional, **keywords)
    elapsed = time.monotonic() - start
    frames = [{"timestamp_seconds": obs.time().toSec(),
               "corners": len(obs.getCornersImageFrame())}
              for obs in observations]
    row = {"topic": dataset.topic, "input_images": dataset.numImages(),
           "accepted_images": len(observations),
           "accepted_corners": sum(f["corners"] for f in frames),
           "extraction_seconds": elapsed, "frames": frames}
    counts["extractions"].append(row)
    save()
    print("OBSERVATION_COUNTS " + json.dumps({k: v for k, v in row.items() if k != "frames"}), flush=True)
    return observations

kc.extractCornersFromDataset = extract

if Path(args.kalibr_script).name == "kalibr_calibrate_imu_camera":
    import kalibr_imu_camera_calibration.IccSensors as sensors
    original_add = sensors.IccCamera.addCameraErrorTerms

    def add(self, *positional, **keywords):
        result = original_add(self, *positional, **keywords)
        groups = self.allReprojectionErrors
        row = {"topic": self.dataset.topic,
               "detected_images": len(self.targetObservations),
               "images_in_spline_bounds": len(groups),
               "images_with_residuals": sum(bool(group) for group in groups),
               "corner_residuals": sum(len(group) for group in groups),
               "images_outside_spline_bounds": len(self.targetObservations) - len(groups)}
        counts["camera_error_terms"].append(row)
        save()
        print("OPTIMIZATION_COUNTS " + json.dumps(row), flush=True)
        return result

    sensors.IccCamera.addCameraErrorTerms = add

sys.argv = [args.kalibr_script] + remaining
save()
runpy.run_path(args.kalibr_script, run_name="__main__")
