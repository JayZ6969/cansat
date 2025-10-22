"use client"

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { MultiAxisChart } from "@/components/charts/multi-axis-chart"
import { RotateCw } from "lucide-react"

interface ReactionWheelSectionProps {
  data: Array<{ time: number; x: number; y: number; z: number }>
}

export function ReactionWheelSection({ data }: ReactionWheelSectionProps) {
  const series = [
    { key: "x", name: "X-axis", color: "hsl(var(--chart-1))" },
    { key: "y", name: "Y-axis", color: "hsl(var(--chart-2))" },
    { key: "z", name: "Z-axis", color: "hsl(var(--chart-3))" },
  ]

  return (
    <Card className="h-full flex flex-col">
      <CardHeader className="pb-3">
        <CardTitle className="text-lg flex items-center gap-2">
          <RotateCw className="h-5 w-5" />
          Reaction Wheel
        </CardTitle>
        <CardDescription>Angular momentum</CardDescription>
      </CardHeader>
      <CardContent className="flex-1 min-h-0">
        <MultiAxisChart
          data={data}
          series={series}
          title=""
          subtitle=""
        />
      </CardContent>
    </Card>
  )
}
