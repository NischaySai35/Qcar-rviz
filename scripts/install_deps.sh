#!/bin/bash
# One-time setup for the object-detection + voice-command features.
#
#   scripts/install_deps.sh
#
# Safe to re-run: every step checks first and skips work already done.
#
# IMPORTANT -- why every pip call here uses /usr/bin/python3 and --no-deps:
#   * Bare `python3` on this car is pyenv's 3.7.17, NOT the system 3.8.10 that
#     rclpy's C extension is built against. Installing into it would produce
#     packages the ROS nodes cannot import. Same trap the node shebangs warn
#     about; see image_preview_throttle.py.
#   * torch here is NVIDIA's Jetson build (2.1.0a0+...nv23.06) and is the only
#     one on this machine with working CUDA. A dependency resolver is happy to
#     "upgrade" it to a generic aarch64 wheel with no CUDA, which would silently
#     drop object detection to CPU. --no-deps makes that impossible.
set -e
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY=/usr/bin/python3
MODELS_DIR="$PROJECT_DIR/models"
mkdir -p "$MODELS_DIR"

echo "[install_deps] Using $($PY -V 2>&1) at $PY"

have() { $PY -c "import $1" >/dev/null 2>&1; }

# --- 1. object detection ----------------------------------------------------
# ultralytics / torch / torchvision / cv2 ship with this JetPack image already.
for mod in torch ultralytics cv2; do
  if ! have "$mod"; then
    echo "[install_deps] ERROR: '$mod' is missing from $PY and is expected to be" >&2
    echo "               part of this JetPack image. Not installing it here --" >&2
    echo "               a generic wheel would likely break CUDA. Investigate first." >&2
    exit 1
  fi
done
echo "[install_deps] torch/ultralytics/cv2 present."
$PY - <<'PY'
import torch
print(f"[install_deps] torch {torch.__version__}  CUDA available: {torch.cuda.is_available()}")
if not torch.cuda.is_available():
    print("[install_deps] WARNING: CUDA is NOT available -- detection will run on CPU and be far too slow.")
PY

# CLIP text encoder: required by YOLO-World's set_classes() to embed the
# free-text vocabulary. Without it only fixed COCO classes would work.
if have clip; then
  echo "[install_deps] clip already installed."
else
  echo "[install_deps] Installing CLIP text encoder..."
  $PY -m pip install --no-deps "git+https://github.com/ultralytics/CLIP.git"
  $PY -m pip install --no-deps ftfy regex
fi

# --- 2. voice commands ------------------------------------------------------
if have vosk && have sounddevice; then
  echo "[install_deps] vosk + sounddevice already installed."
else
  echo "[install_deps] Installing vosk + sounddevice..."
  # libportaudio2 is sounddevice's runtime library; apt only if actually missing.
  if ! ldconfig -p | grep -q libportaudio; then
    sudo apt-get install -y libportaudio2
  fi
  $PY -m pip install --no-deps vosk sounddevice cffi srt
fi

# --- 3. model weights -------------------------------------------------------
YOLO_WEIGHTS="$MODELS_DIR/yolov8s-worldv2.pt"
if [ -f "$YOLO_WEIGHTS" ]; then
  echo "[install_deps] YOLO-World weights present: $YOLO_WEIGHTS"
else
  echo "[install_deps] Downloading YOLO-World weights..."
  curl -fL --retry 3 -o "$YOLO_WEIGHTS" \
    https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8s-worldv2.pt
fi

# CLIP's own ViT-B/32 weights (~338 MB) are fetched to ~/.cache/clip on the
# first set_classes() call. Pull them now so the first mapping run is not
# stalled by a download with the car already driving.
$PY - <<'PY'
import os
cache = os.path.expanduser('~/.cache/clip/ViT-B-32.pt')
if os.path.exists(cache):
    print(f"[install_deps] CLIP weights present: {cache}")
else:
    print("[install_deps] Fetching CLIP ViT-B/32 weights (~338 MB, one time)...")
    import clip
    clip.load('ViT-B/32', device='cpu')
    print("[install_deps] CLIP weights cached.")
PY

# Vosk speech model. "small" is the right trade here: ~40 MB, streams in real
# time on the Orin's CPU, and we constrain it to a command grammar anyway, so a
# larger general-purpose model would cost latency for no accuracy we would use.
VOSK_DIR="$MODELS_DIR/vosk-model-small-en-us-0.15"
if [ -d "$VOSK_DIR" ]; then
  echo "[install_deps] Vosk model present: $VOSK_DIR"
else
  echo "[install_deps] Downloading Vosk English model (~40 MB)..."
  TMP_ZIP="$(mktemp /tmp/vosk-model.XXXXXX.zip)"
  curl -fL --retry 3 -o "$TMP_ZIP" \
    https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
  unzip -q "$TMP_ZIP" -d "$MODELS_DIR"
  rm -f "$TMP_ZIP"
  echo "[install_deps] Vosk model installed."
fi

# --- 4. microphone ----------------------------------------------------------
echo "[install_deps] Checking for a capture device..."
if pactl list short sources 2>/dev/null | grep -v '\.monitor' | grep -q .; then
  pactl list short sources 2>/dev/null | grep -v '\.monitor' | \
    awk '{print "[install_deps]   source: " $2}'
else
  echo "[install_deps] WARNING: no PulseAudio capture source found."
  echo "               Voice commands from the CAR's mic will not work;"
  echo "               the browser-mic option in the web console still will."
fi

echo
echo "[install_deps] Done."
# detect_objects and sensor_fusion both already default to true, so don't
# print a flag that implies they have to be asked for -- only the opt-out is
# worth showing.
echo "[install_deps] Map it yourself:   scripts/start_mapping.sh          (objects ON by default)"
echo "[install_deps] Let it explore:    scripts/start_mapping_auto.sh my_room"
echo "[install_deps] Plain map, no AI:  scripts/start_mapping.sh detect_objects:=false"
