"""Actual model smoke test on synthetic RGB; no ROS or robot connection."""
import argparse
import json
import time

import numpy as np
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', choices=('cpu', 'cuda'), default='cuda')
    parser.add_argument('--checkpoint', default='/workspace/results/models/sam2.1_hiera_tiny.pt')
    args = parser.parse_args()
    if args.device == 'cuda':
        assert torch.cuda.is_available(), 'CUDA is not available'
    torch.set_num_threads(2)

    def synchronize():
        if args.device == 'cuda':
            torch.cuda.synchronize()

    started = time.perf_counter()
    with torch.inference_mode():
        model = build_sam2('configs/sam2.1/sam2.1_hiera_t.yaml', args.checkpoint, device=args.device)
        predictor = SAM2ImagePredictor(model)
        rgb = np.full((480, 640, 3), 220, np.uint8)
        rgb[140:340, 180:460] = [160, 110, 65]
        durations = []
        for _ in range(3):
            synchronize()
            begin = time.perf_counter()
            predictor.set_image(rgb)
            masks, scores, _ = predictor.predict(
                point_coords=np.array([[320, 240]], np.float32), point_labels=np.array([1]),
                box=np.array([180, 140, 460, 340]), multimask_output=True)
            synchronize()
            durations.append(round(time.perf_counter()-begin, 3))
            assert masks.shape == (3, 480, 640) and np.isfinite(scores).all()
        assert next(model.parameters()).device.type == args.device
    print(json.dumps(dict(device=args.device, torch=torch.__version__,
        gpu=torch.cuda.get_device_name(0) if args.device == 'cuda' else None,
        inference_seconds=durations, total_seconds=round(time.perf_counter()-started, 3),
        synthetic_only=True), indent=2))


if __name__ == '__main__':
    main()
