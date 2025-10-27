# 🗺️ Quick Setup: Satellite Maps

## One-Time Setup (After Cloning)

### Windows
```bash
setup_satellite_tiles.bat
```

### macOS/Linux
```bash
chmod +x setup_satellite_tiles.sh
./setup_satellite_tiles.sh
```

### Manual Setup
```bash
python scripts/download_satellite_tiles.py
```

## What This Does

✅ Downloads satellite imagery for mission area  
✅ Center: 26.720333°N, 84.303806°E  
✅ Radius: 3 km  
✅ Time: 5-15 minutes  
✅ Size: ~100-300 MB  

## Result

🛰️ Offline satellite maps  
📍 High-resolution imagery  
🚀 Competition-ready GCS  

## See Also

- [SATELLITE_IMAGERY_SETUP.md](SATELLITE_IMAGERY_SETUP.md) - Detailed guide
- [SATELLITE_MAP_IMPLEMENTATION.md](SATELLITE_MAP_IMPLEMENTATION.md) - Technical details
