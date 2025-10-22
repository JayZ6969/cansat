"use client"

import type { TelemetryData } from "@/lib/telemetry-types"
import { cn } from "@/lib/utils"

interface MissionProgressProps {
  telemetry?: TelemetryData
}

const MISSION_STAGES = [
  { id: 0, name: "BOOT", label: "System Boot" },
  { id: 1, name: "TEST_MODE", label: "Test Mode" },
  { id: 2, name: "LAUNCH_PAD", label: "Launch Pad" },
  { id: 3, name: "ASCENT", label: "Ascent" },
  { id: 4, name: "ROCKET_DEPLOY", label: "Rocket Deploy" },
  { id: 5, name: "DESCENT", label: "Descent" },
  { id: 6, name: "AEROBRAKE_RELEASE", label: "Aerobrake Release" },
  { id: 7, name: "IMPACT", label: "Impact" },
]

export function MissionProgress({ telemetry }: MissionProgressProps) {
  const currentStageId = telemetry?.flightState ?? 0
  const totalStages = MISSION_STAGES.length
  const progressPercentage = ((currentStageId + 1) / totalStages) * 100

  return (
    <div className="fixed bottom-0 left-0 right-0 z-50 bg-card border-t border-border shadow-lg">
      <div className="px-8 py-4">
        {/* Current Stage Label */}
        <div className="flex items-center justify-between mb-3">
          <div className="text-sm font-semibold text-muted-foreground">
            Mission Progress
          </div>
          <div className="text-base font-bold text-foreground">
            {MISSION_STAGES[currentStageId]?.label || "Unknown"}
          </div>
          <div className="text-sm font-semibold text-muted-foreground">
            {currentStageId + 1} / {totalStages}
          </div>
        </div>

        {/* Progress Bar */}
        <div className="relative h-3 bg-muted rounded-full overflow-hidden">
          {/* Filled Progress */}
          <div
            className="absolute top-0 left-0 h-full bg-linear-to-r from-blue-600 to-blue-500 transition-all duration-500 ease-out"
            style={{ width: `${progressPercentage}%` }}
          >
            {/* Animated shine effect */}
            <div className="absolute inset-0 bg-linear-to-r from-transparent via-white/20 to-transparent animate-shimmer" />
          </div>

          {/* Stage Markers */}
          {MISSION_STAGES.map((stage, index) => {
            const markerPosition = ((stage.id + 1) / totalStages) * 100

            return (
              <div
                key={stage.id}
                className="absolute top-0 h-full w-[2px] bg-background"
                style={{ left: `${markerPosition}%` }}
              />
            )
          })}
        </div>

        {/* Stage Labels */}
        <div className="relative mt-2 flex justify-between">
          {MISSION_STAGES.map((stage) => {
            const isCompleted = stage.id < currentStageId
            const isCurrent = stage.id === currentStageId

            return (
              <div
                key={stage.id}
                className={cn(
                  "text-xs font-medium transition-colors",
                  isCurrent && "text-blue-600 font-bold",
                  isCompleted && "text-green-600",
                  !isCurrent && !isCompleted && "text-muted-foreground/60"
                )}
                style={{ flex: 1, textAlign: "center" }}
              >
                {stage.label}
              </div>
            )
          })}
        </div>
      </div>
    </div>
  )
}
