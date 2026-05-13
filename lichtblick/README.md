# Lichtblick Visualization Stack

This directory contains a standalone Docker Compose setup for Lichtblick visualization,
its API, and extension packaging.

## Usage

1. Start your ROS devcontainer in VS Code as usual.
2. In a separate terminal, run:

```bash
cd lichtblick
cp .env.example .env
docker compose up --build
```

After startup:
- Lichtblick web UI: <http://localhost:8080>
- Lichtblick API: <http://localhost:8766>

## Optional: Rebuild Extensions

If you changed files in `lichtblick/extensions`, rebuild extension bundles with:

```bash
cd lichtblick
chmod +x scripts/buildExtensions.sh
docker compose --profile extensions run --rm extension-builder
```

## Notes

- `extension-builder` packages all extensions from `lichtblick/extensions` into
  `lichtblick/workspaces/ufil/extensions`.
- API/web startup does not block on extension packaging so `docker compose up` is
  robust even on low-memory machines.
- The default datasource URL points to `ws://localhost:8765`, which should be provided
  by the bridge running inside the devcontainer.
