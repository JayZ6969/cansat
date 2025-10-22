"use client"

import { useEffect, useRef, useState } from "react"
import type { TelemetryData } from "@/lib/telemetry-types"
import { Card } from "@/components/ui/card"

interface GPSMapProps {
  telemetry?: TelemetryData
}

export function GPSMap({ telemetry }: GPSMapProps) {
  const mapRef = useRef<HTMLDivElement>(null)
  const mapInstanceRef = useRef<any>(null)
  const markerRef = useRef<any>(null)
  const polylineRef = useRef<any>(null)
  const [flightPath, setFlightPath] = useState<Array<{ lat: number; lng: number }>>([])
  const [isLoaded, setIsLoaded] = useState(false)

  // Add new position to flight path
  useEffect(() => {
    if (telemetry && telemetry.latitude !== 0 && telemetry.longitude !== 0) {
      setFlightPath((prev) => {
        // Avoid duplicate consecutive points
        const lastPoint = prev[prev.length - 1]
        if (lastPoint && lastPoint.lat === telemetry.latitude && lastPoint.lng === telemetry.longitude) {
          return prev
        }
        return [...prev, { lat: telemetry.latitude, lng: telemetry.longitude }]
      })
    }
  }, [telemetry])

  // Initialize Leaflet map
  useEffect(() => {
    if (typeof window === 'undefined' || !mapRef.current) return
    
    // Dynamically load Leaflet
    const loadLeaflet = async () => {
      try {
        // Load Leaflet CSS
        if (!document.getElementById('leaflet-css')) {
          const link = document.createElement('link')
          link.id = 'leaflet-css'
          link.rel = 'stylesheet'
          link.href = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'
          link.integrity = 'sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY='
          link.crossOrigin = ''
          document.head.appendChild(link)
        }

        // Load Leaflet JS
        const L = await import('leaflet')
        
        // Fix default marker icon issue with webpack
        delete (L.Icon.Default.prototype as any)._getIconUrl
        L.Icon.Default.mergeOptions({
          iconRetinaUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon-2x.png',
          iconUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon.png',
          shadowUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png',
        })

        // Initialize map only once
        if (!mapInstanceRef.current && mapRef.current) {
          // Default center (will update when we get GPS data)
          const defaultCenter: [number, number] = [37.7749, -122.4194]
          
          mapInstanceRef.current = L.map(mapRef.current).setView(defaultCenter, 13)

          // Add OpenStreetMap tiles
          L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors',
            maxZoom: 19,
          }).addTo(mapInstanceRef.current)

          setIsLoaded(true)
        }
      } catch (error) {
        console.error('Failed to load Leaflet:', error)
      }
    }

    loadLeaflet()

    // Cleanup
    return () => {
      if (mapInstanceRef.current) {
        mapInstanceRef.current.remove()
        mapInstanceRef.current = null
      }
    }
  }, [])

  // Update map with flight path
  useEffect(() => {
    if (!isLoaded || !mapInstanceRef.current || flightPath.length === 0) return

    const updateMap = async () => {
      const L = await import('leaflet')

      // Update or create polyline
      if (polylineRef.current) {
        polylineRef.current.setLatLngs(flightPath)
      } else {
        polylineRef.current = L.polyline(flightPath, {
          color: '#3b82f6',
          weight: 3,
          opacity: 0.8,
        }).addTo(mapInstanceRef.current)
      }

      // Update or create marker for current position
      const currentPos = flightPath[flightPath.length - 1]
      if (markerRef.current) {
        markerRef.current.setLatLng([currentPos.lat, currentPos.lng])
      } else {
        markerRef.current = L.marker([currentPos.lat, currentPos.lng])
          .addTo(mapInstanceRef.current)
          .bindPopup('Current Position')
      }

      // Add launch point marker (first point)
      if (flightPath.length === 1) {
        L.circleMarker([flightPath[0].lat, flightPath[0].lng], {
          radius: 8,
          fillColor: '#10b981',
          color: '#fff',
          weight: 2,
          opacity: 1,
          fillOpacity: 0.8,
        })
          .addTo(mapInstanceRef.current)
          .bindPopup('Launch Point')
      }

      // Fit bounds to show all points
      if (flightPath.length > 1) {
        const bounds = L.latLngBounds(flightPath)
        mapInstanceRef.current.fitBounds(bounds, { padding: [50, 50] })
      } else {
        mapInstanceRef.current.setView([currentPos.lat, currentPos.lng], 15)
      }
    }

    updateMap()
  }, [flightPath, isLoaded])

  return (
    <Card className="flex flex-col h-full bg-card">
      <div className="p-4 border-b border-border">
        <h3 className="text-sm font-semibold text-foreground">GPS Map</h3>
        <p className="text-xs text-muted-foreground">Flight path visualization</p>
      </div>
      <div className="flex-1 relative overflow-hidden">
        <div 
          ref={mapRef} 
          className="w-full h-full"
          style={{ minHeight: '400px' }}
        />
        <div className="absolute bottom-4 left-4 text-xs text-muted-foreground bg-card/90 px-2 py-1 rounded shadow-md z-[1000]">
          {flightPath.length} waypoints
        </div>
        {!isLoaded && (
          <div className="absolute inset-0 flex items-center justify-center bg-card/50 backdrop-blur-sm">
            <p className="text-sm text-muted-foreground">Loading map...</p>
          </div>
        )}
      </div>
    </Card>
  )
}
