import { toast } from '@/hooks/use-toast'

export type NotificationType = 'info' | 'success' | 'warning' | 'error'

export interface NotificationData {
  type: NotificationType
  title: string
  description?: string
  duration?: number
}

class NotificationService {
  private static instance: NotificationService

  public static getInstance(): NotificationService {
    if (!NotificationService.instance) {
      NotificationService.instance = new NotificationService()
    }
    return NotificationService.instance
  }

  private getVariantFromType(type: NotificationType) {
    switch (type) {
      case 'error':
        return 'destructive'
      case 'warning':
        return 'destructive'
      case 'success':
        return 'default'
      case 'info':
      default:
        return 'default'
    }
  }

  public show(notification: NotificationData) {
    toast({
      title: notification.title,
      description: notification.description,
      variant: this.getVariantFromType(notification.type),
      duration: notification.duration || 5000,
    })
  }

  public showConnectionError(error: string) {
    this.show({
      type: 'error',
      title: 'Connection Failed',
      description: error,
      duration: 7000,
    })
  }

  public showConnectionSuccess(port: string) {
    this.show({
      type: 'success',
      title: 'Connected',
      description: `Successfully connected to ${port}`,
      duration: 3000,
    })
  }

  public showDisconnected() {
    this.show({
      type: 'info',
      title: 'Disconnected',
      description: 'Serial port disconnected',
      duration: 3000,
    })
  }

  public showLoRaStatus(connected: boolean) {
    this.show({
      type: connected ? 'success' : 'warning',
      title: `LoRa ${connected ? 'Connected' : 'Disconnected'}`,
      description: connected 
        ? 'Receiving telemetry data' 
        : 'No data received for 30+ seconds',
      duration: 4000,
    })
  }

  public showLogMessage(source: 'GCS' | 'CanSat', message: string, level: 'INFO' | 'WARNING' | 'ERROR' | 'STATUS' = 'INFO') {
    const type: NotificationType = level === 'ERROR' ? 'error' : level === 'WARNING' ? 'warning' : 'info'
    
    this.show({
      type,
      title: `${source} ${level}`,
      description: message,
      duration: level === 'ERROR' ? 8000 : 4000,
    })
  }

  public showStatusMessage(message: string) {
    this.show({
      type: 'info',
      title: 'System Status',
      description: message,
      duration: 3000,
    })
  }
}

export default NotificationService.getInstance()