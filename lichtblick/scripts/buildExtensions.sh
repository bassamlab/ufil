#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
EXTENSIONS_DIR="$ROOT_DIR/extensions"
OUTPUT_DIR="$ROOT_DIR/workspaces/ufil/extensions"

echo "[Lichtblick Extensions] Building..."

rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

for extension_dir in "$EXTENSIONS_DIR"/*; do
  [ -d "$extension_dir" ] || continue
  [ -f "$extension_dir/package.json" ] || continue

  cd "$extension_dir"
  rm -f ./*.foxe

  if [ -f package-lock.json ]; then
    npm ci
  else
    npm install
  fi

  npm run package

  publisher="$(node -p 'require("./package.json").publisher')"
  name="$(node -p 'require("./package.json").name')"

  mv ./*.foxe "$OUTPUT_DIR/$publisher.$name.foxe"
done

echo "[Lichtblick Extensions] Build complete."
