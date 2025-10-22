"use client"

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { GPSMap } from "@/components/gps-map"
import { MapPin } from "lucide-react"
import type { TelemetryData } from "@/lib/telemetry-types"

interface MapSectionProps {
  telemetry?: TelemetryData
}

export function MapSection({ telemetry }: MapSectionProps) {
  return (
    <Card className="h-full flex flex-col">
      <CardHeader className="pb-3">
        <CardTitle className="text-lg flex items-center gap-2">
          <MapPin className="h-5 w-5" />
          GPS Map
        </CardTitle>
      </CardHeader>
      <CardContent className="flex-1 min-h-0 p-0">
        <GPSMap telemetry={telemetry} />
      </CardContent>
    </Card>
  )
}
