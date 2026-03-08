# GPU Setup Guide

## Overview
The detection system now supports GPU acceleration using CUDA for faster object detection with YOLO.

## Configuration

GPU detection is enabled by default. You can control it in `config.py`:

```python
class TrackingConfig:
    USE_GPU = True  # Set to False to force CPU usage
```

## Requirements

### 1. NVIDIA GPU with CUDA Support
- Check if you have a compatible NVIDIA GPU
- Verify CUDA is installed: `nvidia-smi`

### 2. PyTorch with CUDA
Install PyTorch with CUDA support:

```bash
# For CUDA 11.8
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# For CUDA 12.1
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Or check https://pytorch.org/get-started/locally/ for your specific setup
```

### 3. Verify Installation
Check if PyTorch can access your GPU:

```python
import torch
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"GPU: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'None'}")
```

## Performance Benefits

With GPU acceleration:
- **Faster detection**: 5-10x faster inference compared to CPU
- **Higher FPS**: Process more frames per second
- **Better real-time performance**: Smoother tracking and control

## Automatic Fallback

The system automatically detects GPU availability:
- If GPU is available and `USE_GPU = True` → Uses GPU
- If GPU is not available → Falls back to CPU
- You'll see a message at startup indicating which device is being used

## Troubleshooting

### "CUDA not available" message
1. Check NVIDIA driver: `nvidia-smi`
2. Reinstall PyTorch with CUDA support (see above)
3. Verify CUDA toolkit is installed

### Out of memory errors
- Use a smaller model: `yolov8n.pt` instead of `yolov8s.pt`
- Reduce frame resolution in `CameraConfig`
- Increase `SKIP_FRAMES` to process fewer frames

### Still using CPU
- Set `USE_GPU = True` in `config.py`
- Verify PyTorch CUDA installation
- Check GPU compatibility

## Model Files

Both model files work with GPU:
- `yolov8n.pt` - Nano model (fastest, recommended for GPU)
- `yolov8s.pt` - Small model (more accurate, requires more GPU memory)
