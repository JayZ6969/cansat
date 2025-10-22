"use client"

import type { ReactNode } from "react"

interface DashboardLayoutProps {
  header: ReactNode
  statusBar: ReactNode
  leftPanel: ReactNode
  centerPanel: ReactNode
  rightPanel: ReactNode
}

export function DashboardLayout({ header, statusBar, leftPanel, centerPanel, rightPanel }: DashboardLayoutProps) {
  return (
    <div className="flex h-screen flex-col bg-background">
      {/* Header */}
      {header}

      {/* Mission Status Bar */}
      {statusBar}

      {/* Main Content Grid */}
      <div className="flex flex-1 overflow-hidden">
        {/* Left Panel - Map & Controls */}
        <div className="w-1/4 border-r border-border overflow-y-auto">{leftPanel}</div>

        {/* Center Panel - Charts */}
        <div className="flex-1 overflow-y-auto">{centerPanel}</div>

        {/* Right Panel - Telemetry & Status */}
        <div className="w-1/4 border-l border-border overflow-y-auto">{rightPanel}</div>
      </div>
    </div>
  )
}
