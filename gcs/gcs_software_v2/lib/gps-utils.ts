/**
 * GPS coordinate validation utilities
 */

/**
 * Check if GPS coordinates are valid for plotting on a map
 * Filters out zero coordinates (0, 0.0, 0.000000, etc.) and out-of-range values
 */
export function isValidGPSCoordinate(lat: number, lng: number): boolean {
  // Filter out zero/near-zero coordinates (handles 0, 0.0, 0.000000, etc.)
  if (Math.abs(lat) < 0.000001 || Math.abs(lng) < 0.000001) {
    return false
  }
  
  // Validate coordinate ranges
  if (lat < -90 || lat > 90 || lng < -180 || lng > 180) {
    return false
  }
  
  return true
}

/**
 * Check if coordinates are effectively zero (including small floating point values)
 */
export function isZeroCoordinate(lat: number, lng: number): boolean {
  return Math.abs(lat) < 0.000001 && Math.abs(lng) < 0.000001
}

/**
 * Format coordinates for display with proper precision
 */
export function formatCoordinate(value: number, precision: number = 6): string {
  return value.toFixed(precision)
}