"use client"

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { TelemetryChart } from "@/components/charts/telemetry-chart"
import { Gauge } from "lucide-react"

interface PressureSectionProps {
  data: Array<{ time: number; value: number }>
}

export function PressureSection({ data }: PressureSectionProps) {
  return (
    <Card className="h-full flex flex-col">
      <CardHeader className="pb-3">
        <CardTitle className="text-lg flex items-center gap-2">
          <Gauge className="h-5 w-5" />
          Pressure
        </CardTitle>
        <CardDescription>Atmospheric pressure</CardDescription>
      </CardHeader>
      <CardContent className="flex-1 min-h-0">
        <TelemetryChart
          data={data}
          title=""
          subtitle=""
          unit="kPa"
          color="hsl(var(--chart-2))"
        />
      </CardContent>
    </Card>
  )
}
