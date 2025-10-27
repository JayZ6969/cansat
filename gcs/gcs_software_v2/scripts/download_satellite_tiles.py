"""
Download and cache satellite imagery tiles for offline use
Target area: 3km radius around coordinates 26.720333, 84.303806
"""

import os
import requests
import time
import math
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

# Configuration
CENTER_LAT = 26.720333
CENTER_LON = 84.303806
RADIUS_KM = 3
ZOOM_LEVELS = [13, 14, 15, 16, 17]  # Different zoom levels for detailed maps

# Tile server configurations - Using multiple sources for redundancy
TILE_SOURCES = {
    'satellite': {
        'url': 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        'name': 'ESRI World Imagery',
        'folder': 'satellite'
    },
    'hybrid': {
        'url': 'https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
        'name': 'Google Hybrid',
        'folder': 'hybrid'
    }
}

# Output directory
BASE_DIR = Path(__file__).parent.parent / 'public' / 'map_tiles'


def deg2num(lat_deg, lon_deg, zoom):
    """Convert lat/lon to tile numbers"""
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)


def get_tile_range(center_lat, center_lon, radius_km, zoom):
    """Calculate tile range for a given radius"""
    # Approximate: 1 degree ≈ 111 km
    lat_offset = radius_km / 111.0
    lon_offset = radius_km / (111.0 * math.cos(math.radians(center_lat)))
    
    # Get corner tiles
    x_min, y_min = deg2num(center_lat + lat_offset, center_lon - lon_offset, zoom)
    x_max, y_max = deg2num(center_lat - lat_offset, center_lon + lon_offset, zoom)
    
    # Ensure min/max are in correct order
    if x_min > x_max:
        x_min, x_max = x_max, x_min
    if y_min > y_max:
        y_min, y_max = y_max, y_min
    
    return x_min, x_max, y_min, y_max


def download_tile(tile_source, zoom, x, y, output_dir):
    """Download a single tile"""
    url_template = TILE_SOURCES[tile_source]['url']
    url = url_template.format(z=zoom, x=x, y=y)
    
    tile_dir = output_dir / tile_source / str(zoom) / str(x)
    tile_dir.mkdir(parents=True, exist_ok=True)
    tile_path = tile_dir / f"{y}.png"
    
    # Skip if already downloaded
    if tile_path.exists():
        return f"Cached: {tile_source}/{zoom}/{x}/{y}"
    
    try:
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'
        }
        response = requests.get(url, headers=headers, timeout=10)
        
        if response.status_code == 200:
            with open(tile_path, 'wb') as f:
                f.write(response.content)
            return f"Downloaded: {tile_source}/{zoom}/{x}/{y}"
        else:
            return f"Failed ({response.status_code}): {tile_source}/{zoom}/{x}/{y}"
    except Exception as e:
        return f"Error: {tile_source}/{zoom}/{x}/{y} - {str(e)}"


def download_tiles_for_area(tile_source='satellite', max_workers=10):
    """Download all tiles for the specified area"""
    print(f"\n{'='*70}")
    print(f"Downloading {TILE_SOURCES[tile_source]['name']} tiles")
    print(f"Center: {CENTER_LAT}, {CENTER_LON}")
    print(f"Radius: {RADIUS_KM} km")
    print(f"Zoom levels: {ZOOM_LEVELS}")
    print(f"{'='*70}\n")
    
    total_tiles = 0
    downloaded_tiles = 0
    
    # Calculate total tiles to download
    for zoom in ZOOM_LEVELS:
        x_min, x_max, y_min, y_max = get_tile_range(CENTER_LAT, CENTER_LON, RADIUS_KM, zoom)
        total_tiles += (x_max - x_min + 1) * (y_max - y_min + 1)
    
    print(f"Total tiles to process: {total_tiles}\n")
    
    # Download tiles
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = []
        
        for zoom in ZOOM_LEVELS:
            x_min, x_max, y_min, y_max = get_tile_range(CENTER_LAT, CENTER_LON, RADIUS_KM, zoom)
            
            print(f"Zoom {zoom}: Tiles ({x_min}-{x_max}, {y_min}-{y_max})")
            
            for x in range(x_min, x_max + 1):
                for y in range(y_min, y_max + 1):
                    future = executor.submit(download_tile, tile_source, zoom, x, y, BASE_DIR)
                    futures.append(future)
            
            # Small delay between zoom levels to avoid rate limiting
            time.sleep(0.5)
        
        # Process results
        for future in as_completed(futures):
            result = future.result()
            if "Downloaded:" in result:
                downloaded_tiles += 1
            print(result)
    
    print(f"\n{'='*70}")
    print(f"Download complete!")
    print(f"Total processed: {total_tiles}")
    print(f"Newly downloaded: {downloaded_tiles}")
    print(f"Already cached: {total_tiles - downloaded_tiles}")
    print(f"Output directory: {BASE_DIR / tile_source}")
    print(f"{'='*70}\n")


def create_tile_info():
    """Create a JSON file with tile information"""
    import json
    
    info = {
        "center": {
            "lat": CENTER_LAT,
            "lon": CENTER_LON
        },
        "radius_km": RADIUS_KM,
        "zoom_levels": ZOOM_LEVELS,
        "tile_sources": {
            key: {
                "name": value['name'],
                "folder": value['folder']
            }
            for key, value in TILE_SOURCES.items()
        },
        "download_date": time.strftime("%Y-%m-%d %H:%M:%S")
    }
    
    info_path = BASE_DIR / 'tile_info.json'
    with open(info_path, 'w') as f:
        json.dump(info, f, indent=2)
    
    print(f"Tile info saved to: {info_path}")


def main():
    """Main function"""
    print("\n" + "="*70)
    print("CanSat GCS - Satellite Imagery Downloader")
    print("="*70)
    
    # Create output directory
    BASE_DIR.mkdir(parents=True, exist_ok=True)
    
    # Download satellite imagery
    download_tiles_for_area('satellite', max_workers=5)
    
    # Optionally download hybrid tiles (satellite + labels)
    print("\nWould you like to download hybrid tiles (satellite + labels)?")
    print("This will take additional time but provides labeled imagery.")
    choice = input("Download hybrid tiles? (y/n): ").lower().strip()
    
    if choice == 'y':
        download_tiles_for_area('hybrid', max_workers=5)
    
    # Create tile info file
    create_tile_info()
    
    print("\n" + "="*70)
    print("All downloads complete!")
    print("The map will now use cached satellite imagery for offline use.")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
