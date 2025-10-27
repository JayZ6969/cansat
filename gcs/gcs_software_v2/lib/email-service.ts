/**
 * EmailJS Service for CanSat Landing Notifications
 * 
 * This service uses EmailJS to send landing notification emails when
 * the CanSat touches down (flight_state === 7).
 * 
 * Setup:
 * 1. Create an account at https://www.emailjs.com/
 * 2. Create an email service (Gmail, Outlook, etc.)
 * 3. Create an email template with the required variables
 * 4. Add credentials to .env.local file
 */

import emailjs from '@emailjs/browser';

// EmailJS configuration from environment variables
const EMAILJS_CONFIG = {
  serviceId: process.env.NEXT_PUBLIC_EMAILJS_SERVICE_ID || '',
  templateId: process.env.NEXT_PUBLIC_EMAILJS_TEMPLATE_ID || '',
  publicKey: process.env.NEXT_PUBLIC_EMAILJS_PUBLIC_KEY || '',
};

export interface TelemetryData {
  team_id?: string;
  timestamp?: string;
  packet_count?: number;
  altitude?: number;
  pressure?: number;
  temperature?: number;
  voltage?: number;
  battery_percentage?: number;
  gnss_lat?: number;
  gnss_long?: number;
  gnss_alt?: number;
  gnss_sats?: number;
  gnss_time?: string;
  gnss_speed?: number;
  flight_state?: number;
  mode?: string;
  rssi?: number;
  [key: string]: any;
}

/**
 * Initialize EmailJS with public key
 */
export function initializeEmailJS(): boolean {
  try {
    if (!EMAILJS_CONFIG.publicKey) {
      console.warn('[EmailJS] Public key not configured. Email notifications disabled.');
      return false;
    }
    
    emailjs.init(EMAILJS_CONFIG.publicKey);
    console.log('[EmailJS] Initialized successfully');
    return true;
  } catch (error) {
    console.error('[EmailJS] Initialization failed:', error);
    return false;
  }
}

/**
 * Check if EmailJS is properly configured
 */
export function isEmailJSConfigured(): boolean {
  return !!(
    EMAILJS_CONFIG.serviceId &&
    EMAILJS_CONFIG.templateId &&
    EMAILJS_CONFIG.publicKey
  );
}

/**
 * Create Google Maps link from coordinates
 */
function createGoogleMapsLink(lat?: number, lon?: number): string {
  if (!lat || !lon) return 'N/A';
  return `https://www.google.com/maps?q=${lat},${lon}&ll=${lat},${lon}&z=15`;
}

/**
 * Format telemetry data for email template
 */
function formatTelemetryForEmail(telemetry: TelemetryData) {
  return {
    // Basic identifiers
    team_id: telemetry.team_id || 'Unknown',
    timestamp: telemetry.timestamp || 'N/A',
    packet_count: telemetry.packet_count || 0,
    
    // Flight data
    altitude: telemetry.altitude?.toFixed(2) || 'N/A',
    pressure: telemetry.pressure?.toFixed(2) || 'N/A',
    temperature: telemetry.temperature?.toFixed(2) || 'N/A',
    voltage: telemetry.voltage?.toFixed(2) || 'N/A',
    battery: telemetry.battery_percentage?.toFixed(0) || 'N/A',
    
    // GPS data
    latitude: telemetry.gnss_lat?.toFixed(6) || 'N/A',
    longitude: telemetry.gnss_long?.toFixed(6) || 'N/A',
    gps_altitude: telemetry.gnss_alt?.toFixed(2) || 'N/A',
    satellites: telemetry.gnss_sats || 0,
    gps_time: telemetry.gnss_time || 'N/A',
    speed: telemetry.gnss_speed?.toFixed(2) || 'N/A',
    
    // State information
    flight_state: telemetry.flight_state || 0,
    mode: telemetry.mode || 'UNKNOWN',
    rssi: telemetry.rssi || 'N/A',
    
    // Google Maps link
    maps_link: createGoogleMapsLink(telemetry.gnss_lat, telemetry.gnss_long),
    
    // Timestamp for email
    email_sent_at: new Date().toLocaleString(),
  };
}

/**
 * Send landing notification email via EmailJS
 * 
 * @param telemetryData - Current telemetry data from the CanSat
 * @returns Promise<boolean> - True if email was sent successfully
 */
export async function sendLandingNotification(
  telemetryData: TelemetryData
): Promise<boolean> {
  try {
    // Check configuration
    if (!isEmailJSConfigured()) {
      console.error('[EmailJS] Service not configured. Cannot send email.');
      console.error('[EmailJS] Missing:', {
        serviceId: !EMAILJS_CONFIG.serviceId,
        templateId: !EMAILJS_CONFIG.templateId,
        publicKey: !EMAILJS_CONFIG.publicKey,
      });
      return false;
    }

    // Format data for email template
    const emailData = formatTelemetryForEmail(telemetryData);
    
    console.log('[EmailJS] Sending landing notification...', {
      mode: emailData.mode,
      flight_state: emailData.flight_state,
      location: `${emailData.latitude}, ${emailData.longitude}`,
    });

    // Send email via EmailJS
    const response = await emailjs.send(
      EMAILJS_CONFIG.serviceId,
      EMAILJS_CONFIG.templateId,
      emailData
    );

    if (response.status === 200) {
      console.log('[EmailJS] ✅ Landing notification sent successfully!', response);
      return true;
    } else {
      console.error('[EmailJS] ❌ Failed to send notification:', response);
      return false;
    }
  } catch (error) {
    console.error('[EmailJS] ❌ Error sending landing notification:', error);
    return false;
  }
}

/**
 * Send test email with sample data
 */
export async function sendTestEmail(): Promise<boolean> {
  const testData: TelemetryData = {
    team_id: '2024-ASI-CANSAT-TEST',
    timestamp: '12:34:56',
    packet_count: 999,
    altitude: 125.5,
    pressure: 1013.25,
    temperature: 24.5,
    voltage: 7.2,
    battery_percentage: 85,
    gnss_lat: 26.720333,
    gnss_long: 84.303806,
    gnss_alt: 124.8,
    gnss_sats: 8,
    gnss_time: '10:34:22',
    gnss_speed: 0.5,
    flight_state: 7,
    mode: 'TEST - IMPACT',
    rssi: -65,
  };

  return sendLandingNotification(testData);
}

/**
 * Get current EmailJS configuration status
 */
export function getEmailJSStatus() {
  return {
    configured: isEmailJSConfigured(),
    serviceId: EMAILJS_CONFIG.serviceId ? '✓ Set' : '✗ Missing',
    templateId: EMAILJS_CONFIG.templateId ? '✓ Set' : '✗ Missing',
    publicKey: EMAILJS_CONFIG.publicKey ? '✓ Set' : '✗ Missing',
  };
}
