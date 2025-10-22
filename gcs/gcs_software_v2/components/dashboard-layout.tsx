"use client"

import type { ReactNode } from "react"

interface DashboardLayoutProps {
  header: ReactNode
  leftPanel: ReactNode
  centerPanel: ReactNode
  rightPanel: ReactNode
}

export function DashboardLayout({ header, leftPanel, centerPanel, rightPanel }: DashboardLayoutProps) {
  return (
    <div className="flex h-screen flex-col bg-background">
      {/* Header */}
      {header}

      {/* Main Content Grid */}
      <div className="flex flex-1 overflow-hidden">
        {/* Left Panel - Map & Controls */}
        <div className="w-1/4 border-r border-border overflow-y-auto">{leftPanel}</div>

        {/* Center Panel - Charts */}
        <div className="flex-1 overflow-y-auto">{centerPanel}</div>

        {/* Right Panel - Telemetry & Status */}
        <div className="w-1/5 border-l border-border overflow-y-auto">{rightPanel}</div>
      </div>
    </div>
  )
}
