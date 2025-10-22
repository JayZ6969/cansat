"use client"

import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from "recharts"
import { Card } from "@/components/ui/card"

interface TelemetryChartProps {
  title: string
  subtitle: string
  data: Array<{ time: number; value: number }>
  color: string
  unit?: string
}

export function TelemetryChart({ title, subtitle, data, color, unit }: TelemetryChartProps) {
  return (
    <Card className="p-4 bg-card">
      <div className="mb-4">
        <h3 className="text-sm font-semibold text-foreground">{title}</h3>
        <p className="text-xs text-muted-foreground">{subtitle}</p>
      </div>
      <ResponsiveContainer width="100%" height={200}>
        <LineChart data={data} margin={{ top: 5, right: 10, left: -20, bottom: 5 }}>
          <CartesianGrid strokeDasharray="3 3" stroke="var(--color-border)" />
          <XAxis
            dataKey="time"
            tick={{ fontSize: 12 }}
            stroke="var(--color-muted-foreground)"
            tickFormatter={(value) => `${(value / 1000).toFixed(0)}s`}
          />
          <YAxis tick={{ fontSize: 12 }} stroke="var(--color-muted-foreground)" />
          <Tooltip
            contentStyle={{
              backgroundColor: "var(--color-card)",
              border: "1px solid var(--color-border)",
              borderRadius: "0.5rem",
            }}
            formatter={(value: number) => [`${value.toFixed(2)} ${unit || ""}`, "Value"]}
            labelFormatter={(label) => `${(label / 1000).toFixed(1)}s`}
          />
          <Line type="monotone" dataKey="value" stroke={color} dot={false} strokeWidth={2} isAnimationActive={false} />
        </LineChart>
      </ResponsiveContainer>
    </Card>
  )
}
