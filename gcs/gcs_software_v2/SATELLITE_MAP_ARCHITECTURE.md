# Satellite Map System Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    CanSat GCS Application                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌───────────────────────────────────────────────────┐     │
│  │          GPS Map Component (Leaflet.js)           │     │
│  │                                                    │     │
│  │  ┌──────────────────────────────────────────┐    │     │
│  │  │    Intelligent Tile Loading System       │    │     │
│  │  │                                          │    │     │
│  │  │  1. Try Local Cached Tiles First        │    │     │
│  │  │     ↓                                    │    │     │
│  │  │  2. On Error → Switch to Online         │    │     │
│  │  │     ↓                                    │    │     │
│  │  │  3. Add Labels Overlay                  │    │     │
│  │  └──────────────────────────────────────────┘    │     │
│  │                                                    │     │
│  │  Center: 26.720333°N, 84.303806°E                │     │
│  │  Zoom: 13-17 (3km radius)                        │     │
│  └───────────────────────────────────────────────────┘     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                          │
                          ↓
        ┌─────────────────────────────────────┐
        │      Tile Source Selection          │
        └─────────────────────────────────────┘
                          │
        ┌─────────────────┴─────────────────┐
        │                                   │
        ↓                                   ↓
┌──────────────────┐              ┌──────────────────┐
│  LOCAL CACHED    │              │  ONLINE FALLBACK │
│  (Offline)       │              │  (Internet req.) │
├──────────────────┤              ├──────────────────┤
│                  │              │                  │
│ public/          │              │ ESRI ArcGIS      │
│ map_tiles/       │              │ server.arcgis    │
│ satellite/       │              │ online.com       │
│  ├─ 13/          │              │                  │
│  ├─ 14/          │              │ Real-time        │
│  ├─ 15/          │              │ imagery          │
│  ├─ 16/          │              │                  │
│  └─ 17/          │              └──────────────────┘
│                  │                       │
│ ~100-300 MB      │                       │
│ Downloaded once  │                       │
│                  │                       │
└──────────────────┘                       │
         ↑                                 │
         │                                 │
         │                                 │
    ┌────┴────────┐                       │
    │   Download  │                       │
    │   Script    │                       │
    └─────────────┘                       │
         ↑                                 │
         │                                 │
┌────────┴─────────┐                      │
│  First-Time      │                      │
│  Setup           │                      │
├──────────────────┤                      │
│                  │                      │
│ Windows:         │                      │
│ setup_satellite_ │                      │
│ tiles.bat        │                      │
│                  │                      │
│ macOS/Linux:     │                      │
│ setup_satellite_ │                      │
│ tiles.sh         │                      │
│                  │                      │
│ Manual:          │                      │
│ python scripts/  │                      │
│ download_        │                      │
│ satellite_       │                      │
│ tiles.py         │                      │
└──────────────────┘                      │
                                          │
                                          ↓
                               ┌──────────────────┐
                               │  Labels Overlay  │
                               ├──────────────────┤
                               │                  │
                               │ CartoDB Labels   │
                               │ (Always Online)  │
                               │                  │
                               │ Place names,     │
                               │ roads, features  │
                               │                  │
                               └──────────────────┘
```

## Download Process Flow

```
User runs setup script
         │
         ↓
┌─────────────────────┐
│ Calculate tile      │
│ coordinates for:    │
│ • Center point      │
│ • 3km radius        │
│ • Zoom levels 13-17 │
└─────────────────────┘
         │
         ↓
┌─────────────────────┐
│ For each zoom level │
│ and tile position:  │
└─────────────────────┘
         │
         ↓
┌─────────────────────┐     ┌──────────────┐
│ Check if tile       │────→│ Tile exists? │
│ already exists      │     └──────────────┘
└─────────────────────┘            │
                                   │
                    ┌──────────────┴──────────────┐
                    │                             │
                    ↓ No                       Yes↓
         ┌─────────────────────┐      ┌─────────────────┐
         │ Download from       │      │ Skip (already   │
         │ ESRI server         │      │ cached)         │
         └─────────────────────┘      └─────────────────┘
                    │                             │
                    ↓                             │
         ┌─────────────────────┐                 │
         │ Save to public/     │                 │
         │ map_tiles/satellite/│                 │
         │ {z}/{x}/{y}.png     │                 │
         └─────────────────────┘                 │
                    │                             │
                    └─────────────┬───────────────┘
                                  ↓
                        ┌─────────────────┐
                        │ Progress update │
                        └─────────────────┘
                                  │
                                  ↓
                        ┌─────────────────┐
                        │ All tiles done? │
                        └─────────────────┘
                                  │
                    ┌─────────────┴─────────────┐
                    │                           │
                 No │                        Yes│
                    ↓                           ↓
         ┌─────────────────┐         ┌─────────────────┐
         │ Continue to     │         │ Create          │
         │ next tile       │         │ tile_info.json  │
         └─────────────────┘         └─────────────────┘
                                              │
                                              ↓
                                     ┌─────────────────┐
                                     │ Setup complete! │
                                     │ Ready for       │
                                     │ offline use     │
                                     └─────────────────┘
```

## Map Loading Flow (Runtime)

```
Application starts
         │
         ↓
┌─────────────────────┐
│ Initialize Leaflet  │
│ map component       │
└─────────────────────┘
         │
         ↓
┌─────────────────────┐
│ Request tile:       │
│ /map_tiles/         │
│ satellite/15/       │
│ 25630/16123.png     │
└─────────────────────┘
         │
         ↓
┌─────────────────────┐     ┌──────────────┐
│ Next.js static      │────→│ File exists? │
│ file handler        │     └──────────────┘
└─────────────────────┘            │
                                   │
                    ┌──────────────┴──────────────┐
                    │                             │
                 Yes│                          No │
                    ↓                             ↓
         ┌─────────────────────┐      ┌─────────────────┐
         │ Serve cached        │      │ Trigger         │
         │ PNG file            │      │ 'tileerror'     │
         │ (instant)           │      │ event           │
         └─────────────────────┘      └─────────────────┘
                    │                             │
                    │                             ↓
                    │              ┌─────────────────────┐
                    │              │ Switch to online    │
                    │              │ ESRI tile server    │
                    │              └─────────────────────┘
                    │                             │
                    │                             ↓
                    │              ┌─────────────────────┐
                    │              │ Download tile       │
                    │              │ from internet       │
                    │              └─────────────────────┘
                    │                             │
                    └─────────────┬───────────────┘
                                  ↓
                        ┌─────────────────┐
                        │ Display tile    │
                        │ on map          │
                        └─────────────────┘
                                  │
                                  ↓
                        ┌─────────────────┐
                        │ Add labels      │
                        │ overlay (online)│
                        └─────────────────┘
```

## Coverage Area Visualization

```
                        N
                        ↑
                        
        ◄───────── 3 km radius ─────────►
        
    ┌───────────────────────────────────┐
    │                                   │
    │        Mission Coverage Area      │
    │                                   │
    │              26.72°N              │
    │          ┌──────────┐             │
    │          │          │             │
W ──┼──────────┤    ✈️    ├────────────┼── E
    │          │          │             │
    │          └──────────┘             │
    │             84.30°E               │
    │                                   │
    │     Cached Satellite Imagery      │
    │                                   │
    └───────────────────────────────────┘
    
                        ↓
                        S

Coverage: ~28 km² (circular area)
Tiles: ~650 files across 5 zoom levels
Storage: 100-300 MB
```

## Tile Numbering System

```
Zoom Level 15 Example:
┌─────┬─────┬─────┬─────┐
│     │     │     │     │
│ x,y │x+1,y│x+2,y│x+3,y│
│     │     │     │     │
├─────┼─────┼─────┼─────┤
│     │     │     │     │
│x,y+1│  🎯 │     │     │  ← Mission area tiles
│     │     │     │     │
├─────┼─────┼─────┼─────┤
│     │     │     │     │
│     │     │     │     │
│     │     │     │     │
└─────┴─────┴─────┴─────┘

Each tile: 256×256 pixels
File format: PNG
Path: satellite/{z}/{x}/{y}.png
```

## Benefits Summary

```
┌─────────────────────────────────────────────┐
│           ONLINE MODE (Before)              │
├─────────────────────────────────────────────┤
│ ❌ Requires internet                        │
│ ❌ Network latency                          │
│ ❌ Server downtime risk                     │
│ ❌ Data usage                               │
│ ✅ Global coverage                          │
└─────────────────────────────────────────────┘
                    │
                    ↓ UPGRADE
                    │
┌─────────────────────────────────────────────┐
│          OFFLINE MODE (After)               │
├─────────────────────────────────────────────┤
│ ✅ Works offline                            │
│ ✅ Instant loading                          │
│ ✅ 100% reliable                            │
│ ✅ No data usage                            │
│ ✅ Mission area coverage                    │
│ ✅ Automatic online fallback                │
└─────────────────────────────────────────────┘
```
