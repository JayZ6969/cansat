"use client"

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { TelemetryChart } from "@/components/charts/telemetry-chart"
import { Thermometer } from "lucide-react"

interface TemperatureSectionProps {
  data: Array<{ time: number; value: number }>
}

export function TemperatureSection({ data }: TemperatureSectionProps) {
  return (
    <Card className="h-full flex flex-col">
      <CardHeader className="pb-3">
        <CardTitle className="text-lg flex items-center gap-2">
          <Thermometer className="h-5 w-5" />
          Temperature
        </CardTitle>
        <CardDescription>Ambient temperature</CardDescription>
      </CardHeader>
      <CardContent className="flex-1 min-h-0">
        <TelemetryChart
          data={data}
          title=""
          subtitle=""
          unit="°C"
          color="hsl(var(--chart-3))"
        />
      </CardContent>
    </Card>
  )
}
