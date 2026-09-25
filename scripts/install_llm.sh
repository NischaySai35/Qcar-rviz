#!/bin/bash
# OPTIONAL: install a small local LLM so the assistant understands free-form
# question wording.
#
#   scripts/install_llm.sh
#
# YOU DO NOT NEED THIS. Without it, qcar2_assistant.py answers questions using
# its rule parser, which already handles the shapes people actually use
# ("is there a cooler", "how many chairs", "how far is the sofa", "where is
# the bed", "what can you see"). The LLM only widens which PHRASINGS are
# understood.
#
# It never supplies facts. Counts and distances are always computed from the
# landmark map and live TF; the model's only job is to turn your sentence into
# a structured query, and its output is validated against the known intents
# before anything is used. A model that invents "there are three chairs" can
# therefore never reach you as an answer.
#
# Everything runs locally on the Orin -- nothing is sent off the car.
set -e
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ~1 GB in 4-bit, a few hundred ms per query on the Orin, and this task is
# just "classify the question", which a small instruct model does well. A
# bigger model would cost seconds per question for no better parsing.
MODEL="${1:-qwen2.5:1.5b-instruct}"

echo "[install_llm] This installs Ollama (a local model runner) and pulls $MODEL."
echo "[install_llm] Disk needed: roughly 2 GB. Nothing leaves the car."
echo

if command -v ollama >/dev/null 2>&1; then
  echo "[install_llm] Ollama already installed: $(ollama --version 2>&1 | head -1)"
else
  echo "[install_llm] Installing Ollama (needs sudo)..."
  # Official installer; detects aarch64 and the Jetson's CUDA.
  curl -fsSL https://ollama.com/install.sh | sh
fi

# The installer normally starts a systemd service; start one ourselves if not.
if ! curl -sf http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
  echo "[install_llm] Starting the Ollama server..."
  (ollama serve >/tmp/ollama.log 2>&1 &) || true
  for _ in $(seq 30); do
    curl -sf http://127.0.0.1:11434/api/tags >/dev/null 2>&1 && break
    sleep 1
  done
fi

if ! curl -sf http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
  echo "[install_llm] ERROR: the Ollama server did not come up. See /tmp/ollama.log" >&2
  echo "[install_llm] The assistant still works without it (rule parser)." >&2
  exit 1
fi

echo "[install_llm] Pulling $MODEL ..."
ollama pull "$MODEL"

echo
echo "[install_llm] Checking the assistant can reach it..."
/usr/bin/python3 - "$MODEL" <<'PY'
import json, sys, urllib.request
model = sys.argv[1]
body = json.dumps({
    'model': model,
    'prompt': ('Convert the question into JSON. Reply with JSON only.\n'
               'Schema: {"intent": one of ["exists","count","distance","where","list","navigate"], '
               '"target": "<object name or empty>"}\n'
               'Known objects: air cooler, sofa, chair\n'
               'Question: do we have any sort of cooling thing in here\nJSON:'),
    'stream': False, 'options': {'temperature': 0.0, 'num_predict': 80},
}).encode()
req = urllib.request.Request('http://127.0.0.1:11434/api/generate', data=body,
                             headers={'Content-Type': 'application/json'})
try:
    with urllib.request.urlopen(req, timeout=60) as r:
        out = json.loads(r.read().decode()).get('response', '')
    print(f'[install_llm] Model replied: {out.strip()[:200]}')
except Exception as exc:
    print(f'[install_llm] WARNING: test query failed: {exc}')
    sys.exit(1)
PY

echo
echo "[install_llm] Done. The assistant picks this up automatically on next launch."
echo "[install_llm] To disable it later without uninstalling:"
echo "    ros2 launch ... use_voice:=true   (and set the assistant's use_llm param false)"
