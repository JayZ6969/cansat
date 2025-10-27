# Satellite Imagery Setup

This project uses cached satellite imagery for offline map viewing during CanSat missions.

## Mission Area

- **Center Coordinates**: 26.720333°N, 84.303806°E
- **Coverage Radius**: 3 km
- **Zoom Levels**: 13-17 (from regional view to detailed close-up)

## First-Time Setup

After cloning this repository, you need to download the satellite imagery tiles:

### Prerequisites

Make sure you have the required Python packages installed:

```bash
pip install requests
```

### Download Satellite Tiles

Run the download script:

```bash
# On Windows
python scripts\download_satellite_tiles.py

# On macOS/Linux
python scripts/download_satellite_tiles.py
```

This will:
1. Download satellite imagery tiles for the mission area
2. Cache them locally in `public/map_tiles/satellite/`
3. Optionally download hybrid tiles (satellite + labels) in `public/map_tiles/hybrid/`
4. Create a `tile_info.json` file with download metadata

**Note**: The initial download may take 5-15 minutes depending on your internet connection. The script uses parallel downloads to speed up the process.

## Tile Sources

The map component uses a fallback system:

1. **Primary**: Local cached tiles (`/map_tiles/satellite/`)
   - Fastest, works offline
   - ESRI World Imagery
   
2. **Fallback**: Online satellite imagery
   - Used if local tiles are not available
   - Requires internet connection
   - ESRI World Imagery (online)

3. **Labels Overlay**: 
   - CartoDB light labels
   - Provides place names and features
   - Always fetched online for the latest data

## Storage Requirements

Approximate storage space needed:

- **Satellite tiles only**: ~50-150 MB (depending on zoom levels)
- **Hybrid tiles (optional)**: Additional ~50-150 MB
- **Total**: ~100-300 MB

## Re-downloading Tiles

If tiles become corrupted or you want to update them:

```bash
# Delete old tiles
rm -rf public/map_tiles/satellite/
rm -rf public/map_tiles/hybrid/

# Re-run the download script
python scripts/download_satellite_tiles.py
```

## Customization

To change the mission area or coverage, edit `scripts/download_satellite_tiles.py`:

```python
# Configuration
CENTER_LAT = 26.720333  # Your latitude
CENTER_LON = 84.303806  # Your longitude
RADIUS_KM = 3           # Coverage radius in kilometers
ZOOM_LEVELS = [13, 14, 15, 16, 17]  # Zoom levels to download
```

## Offline Operation

Once tiles are downloaded:
- The map will work completely offline
- Only the labels overlay requires internet (optional)
- Flight path tracking works offline
- All map features are fully functional

## Troubleshooting

### Tiles Not Loading

1. Check if tiles exist: `public/map_tiles/satellite/`
2. Check browser console for errors
3. Try re-downloading tiles
4. Ensure Next.js dev server is serving static files correctly

### Slow Download

- The script uses rate limiting to avoid server blocks
- Use fewer zoom levels for faster downloads
- Check your internet connection

### Missing Tiles

If some tiles are missing:
- Re-run the download script (it will skip existing tiles)
- Check the `tile_info.json` for download statistics

## Map Attribution

- Satellite Imagery: © ESRI World Imagery
- Labels: © OpenStreetMap contributors, © CARTO
- Map Library: Leaflet.js

## Legal Notice

Satellite imagery is provided by ESRI for visualization purposes. Please review ESRI's terms of service for commercial use restrictions. This caching is intended for offline mission-critical operations only.
