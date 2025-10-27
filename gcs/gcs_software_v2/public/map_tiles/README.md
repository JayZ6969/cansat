# Map Tiles Directory

This directory contains cached satellite imagery tiles for offline map operation.

## Status

⚠️ **Tiles not downloaded yet**

Run the setup script to download satellite imagery:

```bash
# Windows
setup_satellite_tiles.bat

# macOS/Linux
./setup_satellite_tiles.sh
```

## After Download

Once downloaded, this directory will contain:

```
map_tiles/
├── satellite/          # ESRI satellite imagery
│   ├── 13/            # Zoom level 13
│   ├── 14/            # Zoom level 14
│   ├── 15/            # Zoom level 15
│   ├── 16/            # Zoom level 16
│   └── 17/            # Zoom level 17
├── hybrid/            # Satellite + labels (optional)
│   └── ...
└── tile_info.json     # Download metadata
```

## Mission Area

- **Center**: 26.720333°N, 84.303806°E
- **Radius**: 3 km
- **Coverage**: ~100-300 MB

## Note

Satellite tiles are excluded from git (see `.gitignore`).
Each user must download them after cloning the repository.
