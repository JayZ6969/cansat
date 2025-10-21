/**
 * Map Module - Handles GPS tracking and map visualization
 */

class MapManager {
    constructor() {
        this.map = null;
        this.marker = null;
        this.flightPath = [];
        this.flightPathPolyline = null;
        this.isInitialized = false;
        this.defaultLocation = [40.7128, -74.0060]; // New York as default
        this.initializeMap();
    }

    initializeMap() {
        try {
            // Initialize Leaflet map
            this.map = L.map('map', {
                zoomControl: true,
                attributionControl: false
            }).setView(this.defaultLocation, 13);

            // Add light tile layer for contrast with dark dashboard theme
            L.tileLayer('https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png', {
                attribution: '© OpenStreetMap © CartoDB',
                subdomains: 'abcd',
                maxZoom: 19
            }).addTo(this.map);

            // Add custom marker for CanSat
            this.marker = L.marker(this.defaultLocation, {
                icon: this.createCustomIcon()
            }).addTo(this.map);

            // Initialize flight path polyline
            this.flightPathPolyline = L.polyline([], {
                color: '#00bcd4',
                weight: 3,
                opacity: 0.8,
                smoothFactor: 1
            }).addTo(this.map);

            // Add map controls
            this.addMapControls();

            this.isInitialized = true;
            console.log('Map initialized successfully');

        } catch (error) {
            console.error('Failed to initialize map:', error);
            this.showMapError();
        }
    }

    createCustomIcon() {
        return L.divIcon({
            className: 'custom-cansat-marker',
            html: `
                <div style="
                    width: 20px;
                    height: 20px;
                    background: #00bcd4;
                    border: 3px solid #ffffff;
                    border-radius: 50%;
                    box-shadow: 0 0 10px rgba(0, 188, 212, 0.6);
                    animation: pulse 2s infinite;
                "></div>
                <style>
                    @keyframes pulse {
                        0% { transform: scale(1); }
                        50% { transform: scale(1.2); }
                        100% { transform: scale(1); }
                    }
                </style>
            `,
            iconSize: [26, 26],
            iconAnchor: [13, 13]
        });
    }

    addMapControls() {
        // Add custom control buttons
        const controlsDiv = L.DomUtil.create('div', 'leaflet-control-custom');
        controlsDiv.style.background = 'rgba(0, 0, 0, 0.7)';
        controlsDiv.style.padding = '5px';
        controlsDiv.style.borderRadius = '4px';
        controlsDiv.innerHTML = `
            <button onclick="window.mapManager.centerOnCanSat()" style="
                margin: 2px;
                padding: 5px 10px;
                background: #00bcd4;
                color: white;
                border: none;
                border-radius: 3px;
                cursor: pointer;
                font-size: 12px;
            ">📍 Center</button>
            <button onclick="window.mapManager.toggleFlightPath()" style="
                margin: 2px;
                padding: 5px 10px;
                background: #4caf50;
                color: white;
                border: none;
                border-radius: 3px;
                cursor: pointer;
                font-size: 12px;
            ">🛤️ Path</button>
            <button onclick="window.mapManager.clearFlightPath()" style="
                margin: 2px;
                padding: 5px 10px;
                background: #f44336;
                color: white;
                border: none;
                border-radius: 3px;
                cursor: pointer;
                font-size: 12px;
            ">🗑️ Clear</button>
        `;

        const customControl = L.Control.extend({
            onAdd: function(map) {
                return controlsDiv;
            },
            onRemove: function(map) {}
        });

        new customControl({ position: 'topright' }).addTo(this.map);

        // Add scale control
        L.control.scale({
            position: 'bottomleft',
            metric: true,
            imperial: false
        }).addTo(this.map);
    }

    updatePosition(latitude, longitude, altitude = 0) {
        if (!this.isInitialized || !latitude || !longitude) {
            return;
        }

        const newPosition = [latitude, longitude];

        try {
            // Update marker position
            this.marker.setLatLng(newPosition);

            // Add to flight path
            this.flightPath.push({
                lat: latitude,
                lng: longitude,
                alt: altitude,
                timestamp: new Date()
            });

            // Keep only last 1000 points to prevent memory issues
            if (this.flightPath.length > 1000) {
                this.flightPath = this.flightPath.slice(-1000);
            }

            // Update flight path polyline
            const pathCoordinates = this.flightPath.map(point => [point.lat, point.lng]);
            this.flightPathPolyline.setLatLngs(pathCoordinates);

            // Update marker popup with current data
            this.updateMarkerPopup(latitude, longitude, altitude);

        } catch (error) {
            console.error('Error updating map position:', error);
        }
    }

    updateMarkerPopup(latitude, longitude, altitude) {
        const popupContent = `
            <div style="color: #333; font-size: 12px;">
                <strong>CanSat Position</strong><br>
                <strong>Lat:</strong> ${latitude.toFixed(6)}<br>
                <strong>Lng:</strong> ${longitude.toFixed(6)}<br>
                <strong>Alt:</strong> ${altitude.toFixed(2)}m<br>
                <strong>Time:</strong> ${new Date().toLocaleTimeString()}
            </div>
        `;
        
        this.marker.bindPopup(popupContent);
    }

    centerOnCanSat() {
        if (!this.isInitialized || !this.marker) {
            return;
        }

        const position = this.marker.getLatLng();
        this.map.setView(position, this.map.getZoom());
    }

    toggleFlightPath() {
        if (!this.isInitialized || !this.flightPathPolyline) {
            return;
        }

        if (this.map.hasLayer(this.flightPathPolyline)) {
            this.map.removeLayer(this.flightPathPolyline);
        } else {
            this.map.addLayer(this.flightPathPolyline);
        }
    }

    clearFlightPath() {
        if (!this.isInitialized) {
            return;
        }

        this.flightPath = [];
        if (this.flightPathPolyline) {
            this.flightPathPolyline.setLatLngs([]);
        }
    }

    setMapStyle(style = 'dark') {
        if (!this.isInitialized) {
            return;
        }

        // Remove existing tile layers
        this.map.eachLayer((layer) => {
            if (layer instanceof L.TileLayer) {
                this.map.removeLayer(layer);
            }
        });

        // Add new tile layer based on style
        let tileUrl;
        switch (style) {
            case 'satellite':
                tileUrl = 'https://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}';
                break;
            case 'dark':
                tileUrl = 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png';
                break;
            case 'light':
            default:
                tileUrl = 'https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png';
                break;
        }

        L.tileLayer(tileUrl, {
            attribution: '© OpenStreetMap © CartoDB',
            subdomains: style === 'satellite' ? ['mt0', 'mt1', 'mt2', 'mt3'] : 'abcd',
            maxZoom: 19
        }).addTo(this.map);
    }

    addLandingZone(latitude, longitude, radius = 100) {
        if (!this.isInitialized) {
            return;
        }

        const landingZone = L.circle([latitude, longitude], {
            color: '#4caf50',
            fillColor: '#4caf50',
            fillOpacity: 0.2,
            radius: radius
        }).addTo(this.map);

        landingZone.bindPopup(`
            <div style="color: #333;">
                <strong>Landing Zone</strong><br>
                Radius: ${radius}m
            </div>
        `);

        return landingZone;
    }

    addNoFlyZone(latitude, longitude, radius = 200) {
        if (!this.isInitialized) {
            return;
        }

        const noFlyZone = L.circle([latitude, longitude], {
            color: '#f44336',
            fillColor: '#f44336',
            fillOpacity: 0.3,
            radius: radius
        }).addTo(this.map);

        noFlyZone.bindPopup(`
            <div style="color: #333;">
                <strong>No-Fly Zone</strong><br>
                Radius: ${radius}m
            </div>
        `);

        return noFlyZone;
    }

    getFlightStatistics() {
        if (this.flightPath.length < 2) {
            return {
                totalDistance: 0,
                maxAltitude: 0,
                flightTime: 0,
                averageSpeed: 0
            };
        }

        let totalDistance = 0;
        let maxAltitude = 0;
        
        for (let i = 1; i < this.flightPath.length; i++) {
            const prev = this.flightPath[i - 1];
            const curr = this.flightPath[i];
            
            // Calculate distance using Haversine formula
            const distance = this.calculateDistance(
                prev.lat, prev.lng,
                curr.lat, curr.lng
            );
            totalDistance += distance;
            
            maxAltitude = Math.max(maxAltitude, curr.alt);
        }

        const flightTime = (this.flightPath[this.flightPath.length - 1].timestamp - 
                          this.flightPath[0].timestamp) / 1000; // in seconds
        const averageSpeed = flightTime > 0 ? totalDistance / flightTime : 0;

        return {
            totalDistance: totalDistance.toFixed(2),
            maxAltitude: maxAltitude.toFixed(2),
            flightTime: flightTime.toFixed(0),
            averageSpeed: averageSpeed.toFixed(2)
        };
    }

    calculateDistance(lat1, lng1, lat2, lng2) {
        const R = 6371e3; // Earth's radius in meters
        const φ1 = lat1 * Math.PI / 180;
        const φ2 = lat2 * Math.PI / 180;
        const Δφ = (lat2 - lat1) * Math.PI / 180;
        const Δλ = (lng2 - lng1) * Math.PI / 180;

        const a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
                  Math.cos(φ1) * Math.cos(φ2) *
                  Math.sin(Δλ/2) * Math.sin(Δλ/2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

        return R * c;
    }

    showMapError() {
        const mapContainer = document.getElementById('map');
        if (mapContainer) {
            mapContainer.innerHTML = `
                <div style="
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    height: 100%;
                    color: #ff9800;
                    text-align: center;
                    padding: 20px;
                ">
                    <div>
                        <div style="font-size: 24px; margin-bottom: 10px;">⚠️</div>
                        <div>Map unavailable</div>
                        <div style="font-size: 12px; margin-top: 5px;">Check internet connection</div>
                    </div>
                </div>
            `;
        }
    }

    reset() {
        if (!this.isInitialized) {
            return;
        }

        // Reset to default location
        this.marker.setLatLng(this.defaultLocation);
        this.map.setView(this.defaultLocation, 13);
        
        // Clear flight path
        this.clearFlightPath();
    }

    resize() {
        if (this.isInitialized && this.map) {
            setTimeout(() => {
                this.map.invalidateSize();
            }, 100);
        }
    }
}

// Make MapManager available globally
window.MapManager = MapManager;