  "use client"

import { useEffect, useRef, useState } from "react"
import type { TelemetryData } from "@/lib/telemetry-types"
import { Card } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Navigation, MapPin, Rocket } from "lucide-react"
import { isValidGPSCoordinate } from "@/lib/gps-utils"
import "leaflet/dist/leaflet.css"

interface GPSMapProps {
  telemetry?: TelemetryData
}

export function GPSMap({ telemetry }: GPSMapProps) {
  const mapRef = useRef<HTMLDivElement>(null)
  const mapInstanceRef = useRef<any>(null)
  const polylineRef = useRef<any>(null)
  const currentMarkerRef = useRef<any>(null)
  const launchMarkerRef = useRef<any>(null)
  const [flightPath, setFlightPath] = useState<Array<[number, number]>>([])
  const [waypointCount, setWaypointCount] = useState(0)
  const [leafletLoaded, setLeafletLoaded] = useState(false)
  const LRef = useRef<any>(null)

  // Load Leaflet dynamically
  useEffect(() => {
    if (typeof window !== "undefined" && !LRef.current) {
      import("leaflet").then((leaflet) => {
        LRef.current = leaflet.default
        
        // Fix default icon issue with Leaflet in Next.js
        delete (LRef.current.Icon.Default.prototype as any)._getIconUrl
        LRef.current.Icon.Default.mergeOptions({
          iconRetinaUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-icon-2x.png',
          iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-icon.png',
          shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-shadow.png',
        })
        
        setLeafletLoaded(true)
      })
    }
  }, [])

  // Navigation functions
  const goToCurrentPosition = () => {
    console.log('Go to current position clicked', { 
      hasMarker: !!currentMarkerRef.current, 
      hasMap: !!mapInstanceRef.current 
    })
    if (currentMarkerRef.current && mapInstanceRef.current) {
      const pos = currentMarkerRef.current.getLatLng()
      mapInstanceRef.current.setView(pos, 15, { animate: true, duration: 0.5 })
      currentMarkerRef.current.openPopup()
    }
  }

  const goToLaunchPoint = () => {
    console.log('Go to launch point clicked', { 
      hasMarker: !!launchMarkerRef.current, 
      hasMap: !!mapInstanceRef.current 
    })
    if (launchMarkerRef.current && mapInstanceRef.current) {
      const pos = launchMarkerRef.current.getLatLng()
      mapInstanceRef.current.setView(pos, 15, { animate: true, duration: 0.5 })
      launchMarkerRef.current.openPopup()
    }
  }

  const fitAllMarkers = () => {
    console.log('Fit all markers clicked', { 
      pathLength: flightPath.length, 
      hasMap: !!mapInstanceRef.current,
      hasL: !!LRef.current
    })
    if (mapInstanceRef.current && flightPath.length > 0 && LRef.current) {
      const bounds = LRef.current.latLngBounds(flightPath)
      mapInstanceRef.current.fitBounds(bounds, { 
        padding: [50, 50],
        animate: true,
        duration: 0.5
      })
    }
  }

  // Initialize map
  useEffect(() => {
    if (!mapRef.current || mapInstanceRef.current || !leafletLoaded || !LRef.current) return

    // Create map centered on default location
    const map = LRef.current.map(mapRef.current, {
      center: [12.9716, 77.5946], // Default: Bangalore, India
      zoom: 13,
      zoomControl: true,
    })

    // Add OpenStreetMap tile layer
    LRef.current.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '© OpenStreetMap contributors',
      maxZoom: 19,
    }).addTo(map)

    mapInstanceRef.current = map

    return () => {
      if (mapInstanceRef.current) {
        mapInstanceRef.current.remove()
        mapInstanceRef.current = null
      }
    }
  }, [leafletLoaded])

  // Update map with telemetry data
  useEffect(() => {
    if (!telemetry || !mapInstanceRef.current || !LRef.current) return

    const lat = telemetry.latitude
    const lng = telemetry.longitude

    console.log('GPS Map - Received coordinates:', { lat, lng })

    // Filter out invalid coordinates using utility function
    if (!isValidGPSCoordinate(lat, lng)) {
      console.log('GPS Map - Ignoring invalid coordinates:', { lat, lng })
      return
    }

    const newPoint: [number, number] = [lat, lng]

    // Update or create polyline
    if (polylineRef.current) {
      polylineRef.current.addLatLng(newPoint)
    } else {
      polylineRef.current = LRef.current.polyline([newPoint], {
        color: '#3b82f6',
        weight: 3,
        opacity: 0.7,
      }).addTo(mapInstanceRef.current)
    }

    // Update or create current position marker
    if (currentMarkerRef.current) {
      currentMarkerRef.current.setLatLng(newPoint)
    } else {
      const currentIcon = LRef.current.divIcon({
        className: 'custom-marker',
        html: `<div style="background-color: #ef4444; width: 12px; height: 12px; border-radius: 50%; border: 2px solid white; box-shadow: 0 0 4px rgba(0,0,0,0.5);"></div>`,
        iconSize: [12, 12],
        iconAnchor: [6, 6],
      })
      
      currentMarkerRef.current = LRef.current.marker(newPoint, { icon: currentIcon })
        .addTo(mapInstanceRef.current)
        .bindPopup('Current Position')
    }

    // Add launch marker on first valid point (only create if we don't have one and this is a valid coordinate)
    if (flightPath.length === 0 && !launchMarkerRef.current) {
      const launchIcon = LRef.current.divIcon({
        className: 'custom-marker',
        html: `<div style="background-color: #22c55e; width: 10px; height: 10px; border-radius: 50%; border: 2px solid white; box-shadow: 0 0 4px rgba(0,0,0,0.5);"></div>`,
        iconSize: [10, 10],
        iconAnchor: [5, 5],
      })
      
      launchMarkerRef.current = LRef.current.marker(newPoint, { icon: launchIcon })
        .addTo(mapInstanceRef.current)
        .bindPopup('Launch Point')
      
      console.log('GPS Map - Launch point set:', { lat, lng })
    }

    // Add to flight path and update count
    setFlightPath((prev) => {
      const updated = [...prev, newPoint]
      setWaypointCount(updated.length)
      return updated
    })

    // Center map on current position
    mapInstanceRef.current.setView(newPoint, mapInstanceRef.current.getZoom())

    // Fit bounds to show entire flight path (only if we have multiple points)
    if (polylineRef.current) {
      const bounds = polylineRef.current.getBounds()
      if (bounds.isValid()) {
        mapInstanceRef.current.fitBounds(bounds, { padding: [50, 50] })
      }
    }
  }, [telemetry])

  return (
    <Card className="p-4 h-full flex flex-col">
      <div className="mb-3 flex items-center justify-between shrink-0">
        <div>
          <h3 className="text-sm font-semibold text-foreground">GPS Map</h3>
          <p className="text-xs text-muted-foreground">Flight path visualization</p>
        </div>
        <div className="rounded bg-muted px-2 py-1 text-xs font-medium text-muted-foreground">
          {waypointCount} waypoints
        </div>
      </div>
      <div 
        ref={mapRef} 
        className="flex-1 w-full rounded-md border border-border overflow-hidden mb-3"
        style={{ background: '#f1f5f9' }}
      />
      {/* Navigation Controls */}
      <div className="flex gap-2 justify-center shrink-0">
        <Button
          size="sm"
          variant="outline"
          className="flex items-center gap-2"
          onClick={goToCurrentPosition}
          disabled={!currentMarkerRef.current}
        >
          <Navigation className="h-4 w-4 text-red-500" />
          <span className="text-xs">Current Position</span>
        </Button>
        <Button
          size="sm"
          variant="outline"
          className="flex items-center gap-2"
          onClick={goToLaunchPoint}
          disabled={!launchMarkerRef.current}
        >
          <Rocket className="h-4 w-4 text-green-500" />
          <span className="text-xs">Launch Point</span>
        </Button>
        <Button
          size="sm"
          variant="outline"
          className="flex items-center gap-2"
          onClick={fitAllMarkers}
          disabled={flightPath.length === 0}
        >
          <MapPin className="h-4 w-4" />
          <span className="text-xs">View All</span>
        </Button>
      </div>
    </Card>
  )
}
