"use client"

import { Card } from "@/components/ui/card"

interface GaugeChartProps {
  title: string
  value: number
  max: number
  unit: string
  color: string
}

export function GaugeChart({ title, value, max, unit, color }: GaugeChartProps) {
  const percentage = (value / max) * 100

  return (
    <Card className="p-4 bg-card flex flex-col items-center justify-center">
      <h3 className="text-sm font-semibold text-foreground mb-4">{title}</h3>

      <div className="relative w-32 h-32 mb-4">
        <svg viewBox="0 0 100 100" className="w-full h-full">
          {/* Background arc */}
          <path
            d="M 20 80 A 40 40 0 0 1 80 80"
            fill="none"
            stroke="var(--color-border)"
            strokeWidth="8"
            strokeLinecap="round"
          />
          {/* Value arc */}
          <path
            d="M 20 80 A 40 40 0 0 1 80 80"
            fill="none"
            stroke={color}
            strokeWidth="8"
            strokeLinecap="round"
            strokeDasharray={`${(percentage / 100) * 188} 188`}
            style={{ transition: "stroke-dasharray 0.3s ease" }}
          />
          {/* Center text */}
          <text x="50" y="55" textAnchor="middle" fontSize="20" fontWeight="bold" fill="var(--color-foreground)">
            {value.toFixed(0)}
          </text>
          <text x="50" y="70" textAnchor="middle" fontSize="12" fill="var(--color-muted-foreground)">
            {unit}
          </text>
        </svg>
      </div>

      <div className="text-center">
        <p className="text-xs text-muted-foreground">
          {percentage.toFixed(0)}% of {max} {unit}
        </p>
      </div>
    </Card>
  )
}
