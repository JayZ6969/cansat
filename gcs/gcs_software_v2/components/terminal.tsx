'use client'

import { useState, useEffect, useRef } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Badge } from '@/components/ui/badge'
import { Trash2, Download } from 'lucide-react'

export interface LogEntry {
  id: string
  timestamp: string
  source: 'GCS' | 'CanSat' | 'System'
  level: 'INFO' | 'WARNING' | 'ERROR' | 'STATUS' | 'DEBUG'
  message: string
  raw?: string
}

interface TerminalProps {
  className?: string
  maxLines?: number
}

export function Terminal({ className = '', maxLines = 1000 }: TerminalProps) {
  const [logs, setLogs] = useState<LogEntry[]>([])
  const scrollAreaRef = useRef<HTMLDivElement>(null)
  const [autoScroll, setAutoScroll] = useState(true)

  const addLog = (entry: Omit<LogEntry, 'id' | 'timestamp'>) => {
    const newEntry: LogEntry = {
      ...entry,
      id: Date.now().toString() + Math.random().toString(36).substr(2, 9),
      timestamp: new Date().toLocaleTimeString(),
    }

    setLogs(prev => {
      const updated = [...prev, newEntry]
      if (updated.length > maxLines) {
        return updated.slice(-maxLines)
      }
      return updated
    })
  }

  useEffect(() => {
    if (autoScroll && scrollAreaRef.current) {
      scrollAreaRef.current.scrollTop = scrollAreaRef.current.scrollHeight
    }
  }, [logs, autoScroll])

  const clearLogs = () => {
    setLogs([])
  }

  const exportLogs = () => {
    const logText = logs.map(log => 
      `[${log.timestamp}] ${log.source} ${log.level}: ${log.message}`
    ).join('\n')
    
    const blob = new Blob([logText], { type: 'text/plain' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `gcs-logs-${new Date().toISOString().split('T')[0]}.txt`
    a.click()
    URL.revokeObjectURL(url)
  }

  const getLevelColor = (level: string) => {
    switch (level) {
      case 'ERROR': return 'text-red-400 bg-red-900/20'
      case 'WARNING': return 'text-yellow-400 bg-yellow-900/20'
      case 'STATUS': return 'text-green-400 bg-green-900/20'
      case 'INFO': return 'text-blue-400 bg-blue-900/20'
      case 'DEBUG': return 'text-gray-400 bg-gray-900/20'
      default: return 'text-gray-300 bg-gray-900/20'
    }
  }

  const getSourceColor = (source: string) => {
    switch (source) {
      case 'CanSat': return 'text-emerald-400 bg-emerald-900/20'
      case 'GCS': return 'text-blue-400 bg-blue-900/20'
      case 'System': return 'text-purple-400 bg-purple-900/20'
      default: return 'text-gray-400 bg-gray-900/20'
    }
  }

  // Expose addLog function globally for other components to use
  useEffect(() => {
    (window as any).addTerminalLog = addLog
    return () => {
      delete (window as any).addTerminalLog
    }
  }, [])

  return (
    <Card className={`bg-gray-900 border-gray-700 ${className}`}>
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-sm font-medium text-gray-200">Serial Terminal</CardTitle>
        <div className="flex items-center space-x-2">
          <Badge variant="outline" className="text-xs text-gray-400">
            {logs.length} lines
          </Badge>
          <Button
            variant="outline"
            size="sm"
            onClick={exportLogs}
            disabled={logs.length === 0}
            className="h-7 w-7 p-0"
          >
            <Download className="h-3 w-3" />
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={clearLogs}
            disabled={logs.length === 0}
            className="h-7 w-7 p-0"
          >
            <Trash2 className="h-3 w-3" />
          </Button>
        </div>
      </CardHeader>
      <CardContent className="p-0">
        <ScrollArea 
          className="h-48 w-full p-4 font-mono text-xs"
          ref={scrollAreaRef}
        >
          {logs.length === 0 ? (
            <div className="text-gray-500 text-center py-8">
              No logs yet. Connect to a serial port to see data.
            </div>
          ) : (
            <div className="space-y-1">
              {logs.map((log) => (
                <div key={log.id} className="flex items-start space-x-2 text-gray-300">
                  <span className="text-gray-500 text-xs shrink-0">
                    {log.timestamp}
                  </span>
                  <Badge className={`text-xs px-1 py-0 ${getSourceColor(log.source)}`}>
                    {log.source}
                  </Badge>
                  <Badge className={`text-xs px-1 py-0 ${getLevelColor(log.level)}`}>
                    {log.level}
                  </Badge>
                  <span className="break-all">{log.message}</span>
                </div>
              ))}
            </div>
          )}
        </ScrollArea>
      </CardContent>
    </Card>
  )
}

// Helper function for other components to add logs
export const addTerminalLog = (entry: Omit<LogEntry, 'id' | 'timestamp'>) => {
  if (typeof window !== 'undefined' && (window as any).addTerminalLog) {
    (window as any).addTerminalLog(entry)
  }
}