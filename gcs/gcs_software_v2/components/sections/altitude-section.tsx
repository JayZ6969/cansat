"use client"

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { TelemetryChart } from "@/components/charts/telemetry-chart"
import { Mountain } from "lucide-react"

interface AltitudeSectionProps {
  data: Array<{ time: number; value: number }>
}

export function AltitudeSection({ data }: AltitudeSectionProps) {
  return (
    <Card className="h-full flex flex-col">
      <CardHeader className="pb-3">
        <CardTitle className="text-lg flex items-center gap-2">
          <Mountain className="h-5 w-5" />
          Altitude
        </CardTitle>
        <CardDescription>Height above ground level</CardDescription>
      </CardHeader>
      <CardContent className="flex-1 min-h-0">
        <TelemetryChart
          data={data}
          title=""
          subtitle=""
          unit="m"
          color="hsl(var(--chart-1))"
        />
      </CardContent>
    </Card>
  )
}
