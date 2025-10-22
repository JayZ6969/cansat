"use client"

import { Card } from "@/components/ui/card"

interface GaugeChartProps {
  title: string
  value: number
  min?: number
  max: number
  unit: string
  color?: string
}

export function GaugeChart({ title, value, min = 0, max, unit, color = "hsl(var(--chart-1))" }: GaugeChartProps) {
  // Calculate needle angle (from -135° to +135°, total 270°)
  const percentage = Math.max(0, Math.min(100, ((value - min) / (max - min)) * 100))
  const angle = -135 + (percentage / 100) * 270

  // Generate tick marks
  const ticks = Array.from({ length: 11 }, (_, i) => {
    const tickAngle = -135 + (i / 10) * 270
    const tickValue = min + ((max - min) * i) / 10
    return { angle: tickAngle, value: tickValue }
  })

  return (
    <Card className="p-4 bg-card flex flex-col items-center justify-center">
      <h3 className="text-sm font-semibold text-foreground mb-2">{title}</h3>

      <div className="relative w-48 h-36 mb-2">
        <svg viewBox="0 0 200 150" className="w-full h-full">
          {/* Background arc */}
          <path
            d="M 20 130 A 80 80 0 0 1 180 130"
            fill="none"
            stroke="hsl(var(--border))"
            strokeWidth="12"
            strokeLinecap="round"
          />
          
          {/* Colored segments */}
          <path
            d="M 20 130 A 80 80 0 0 1 180 130"
            fill="none"
            stroke={color}
            strokeWidth="12"
            strokeLinecap="round"
            strokeDasharray={`${(percentage / 100) * 251} 251`}
            style={{ transition: "stroke-dasharray 0.5s ease" }}
            opacity="0.8"
          />

          {/* Tick marks */}
          {ticks.map((tick, i) => {
            const isMainTick = i % 2 === 0
            const tickLength = isMainTick ? 15 : 10
            const x1 = 100 + 75 * Math.cos((tick.angle * Math.PI) / 180)
            const y1 = 130 + 75 * Math.sin((tick.angle * Math.PI) / 180)
            const x2 = 100 + (75 - tickLength) * Math.cos((tick.angle * Math.PI) / 180)
            const y2 = 130 + (75 - tickLength) * Math.sin((tick.angle * Math.PI) / 180)
            
            return (
              <g key={i}>
                <line
                  x1={x1}
                  y1={y1}
                  x2={x2}
                  y2={y2}
                  stroke="hsl(var(--muted-foreground))"
                  strokeWidth={isMainTick ? 2 : 1}
                />
                {isMainTick && (
                  <text
                    x={100 + 55 * Math.cos((tick.angle * Math.PI) / 180)}
                    y={130 + 55 * Math.sin((tick.angle * Math.PI) / 180)}
                    textAnchor="middle"
                    dominantBaseline="middle"
                    fontSize="8"
                    fill="hsl(var(--muted-foreground))"
                  >
                    {tick.value.toFixed(0)}
                  </text>
                )}
              </g>
            )
          })}

          {/* Center pivot circle */}
          <circle cx="100" cy="130" r="8" fill="hsl(var(--card))" stroke={color} strokeWidth="2" />

          {/* Needle */}
          <line
            x1="100"
            y1="130"
            x2={100 + 65 * Math.cos((angle * Math.PI) / 180)}
            y2={130 + 65 * Math.sin((angle * Math.PI) / 180)}
            stroke={color}
            strokeWidth="3"
            strokeLinecap="round"
            style={{ 
              transition: "all 0.5s cubic-bezier(0.4, 0.0, 0.2, 1)",
              transformOrigin: "100px 130px"
            }}
          />

          {/* Needle shadow for depth */}
          <line
            x1="100"
            y1="130"
            x2={100 + 65 * Math.cos((angle * Math.PI) / 180)}
            y2={130 + 65 * Math.sin((angle * Math.PI) / 180)}
            stroke="rgba(0,0,0,0.2)"
            strokeWidth="4"
            strokeLinecap="round"
            style={{ 
              transition: "all 0.5s cubic-bezier(0.4, 0.0, 0.2, 1)",
              transformOrigin: "100px 130px"
            }}
            transform="translate(1, 1)"
          />

          {/* Center pivot highlight */}
          <circle cx="100" cy="130" r="4" fill={color} opacity="0.9" />
        </svg>
      </div>

      <div className="text-center">
        <p className="text-2xl font-bold text-foreground">
          {value.toFixed(1)} <span className="text-sm font-normal text-muted-foreground">{unit}</span>
        </p>
        <p className="text-xs text-muted-foreground mt-1">
          Range: {min} - {max} {unit}
        </p>
      </div>
    </Card>
  )
}
