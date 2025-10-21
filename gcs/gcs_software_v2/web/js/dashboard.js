/**
 * Dashboard Module - Handles dashboard functionality and data display
 */

class Dashboard {
    constructor() {
        this.isPlotting = false;
        this.currentData = {};
        this.alerts = [];
        this.missionStartTime = null;
        this.setupEventListeners();
    }

    setupEventListeners() {
        // Window resize handler for charts
        window.addEventListener('resize', () => {
            if (window.chartsManager) {
                window.chartsManager.resizeCharts();
            }
        });
    }

    updateTelemetryDisplay(data) {
        this.currentData = data;
        
        // Update basic telemetry values
        this.updateElement('mission-time', this.formatTime(data.mission_time || 0));
        this.updateElement('mission-time-sec', data.mission_time || 0);
        this.updateElement('altitude-value', this.formatNumber(data.altitude, 2));
        this.updateElement('altitude-display', this.formatNumber(data.altitude, 2));
        this.updateElement('pressure-value', this.formatNumber(data.pressure, 2));
        this.updateElement('pressure-display', this.formatNumber(data.pressure, 2));
        this.updateElement('temperature-value', this.formatNumber(data.temperature, 1));
        this.updateElement('temperature-display', this.formatNumber(data.temperature, 1));
        this.updateElement('battery-voltage', this.formatNumber(data.battery_voltage, 2));
        this.updateElement('speed-value', this.formatNumber(data.speed, 2));
        this.updateElement('packet-count', data.packet_count || 0);
        this.updateElement('satellites-count', data.num_satellites || 0);
        this.updateElement('error-indicator', data.error_indicator || 'All Good');

        // Determine camera status - auto-set to Recording when data is being received
        const cameraStatus = this.isPlotting && data.packet_count > 0 ? 'Recording' : (data.camera_recording ? 'Recording' : 'Stopped');
        
        // Update status indicators
        this.updateElement('camera-recording', cameraStatus);
        this.updateElement('parachute-deployed', data.parachute_deployed ? 'Deployed' : 'Stowed');
        this.updateElement('calibration-done', data.calibration_done ? 'Done' : 'Pending');
        this.updateElement('current-mode', data.mode || 'Unknown');

        // Update status in header
        this.updateElement('camera-status', cameraStatus);
        this.updateElement('parachute-status', data.parachute_deployed ? 'Deployed' : 'Mode');
        this.updateElement('calibration-status', data.calibration_done ? 'Done' : 'Decent');

        // Update mission phase
        this.updateMissionPhase(data.mode);

        // Apply status colors
        this.applyStatusColors(data);

        // Update charts if plotting is active
        if (this.isPlotting && window.chartsManager) {
            window.chartsManager.updateChart('reactionWheel', data, data.timestamp);
            window.chartsManager.updateChart('gyroscope', data, data.timestamp);
            window.chartsManager.updateChart('altitude', data, data.timestamp);
            window.chartsManager.updateChart('pressure', data, data.timestamp);
            window.chartsManager.updateChart('temperature', data, data.timestamp);
            window.chartsManager.updateChart('accelerometer', data, data.timestamp);
        }

        // Update map if available
        if (window.mapManager && data.gps_latitude && data.gps_longitude) {
            window.mapManager.updatePosition(data.gps_latitude, data.gps_longitude, data.altitude);
        }
    }

    updateElement(id, value, addAnimation = true) {
        const element = document.getElementById(id);
        if (element) {
            element.textContent = value;
            
            if (addAnimation) {
                element.parentElement?.classList.add('updated');
                setTimeout(() => {
                    element.parentElement?.classList.remove('updated');
                }, 500);
            }
        }
    }

    formatNumber(value, decimals = 2) {
        if (typeof value !== 'number') return '0.00';
        return value.toFixed(decimals);
    }

    formatTime(seconds) {
        const mins = Math.floor(seconds / 60);
        const secs = seconds % 60;
        return `${mins}:${secs.toString().padStart(2, '0')}`;
    }

    updateMissionPhase(currentMode) {
        // Map incoming mode strings to phase elements
        const phaseMapping = {
            // Frontend display modes
            'Pre Launch': 'pre-launch',
            'Calibration': 'calibration',
            'Ready to Launch': 'ready',
            'Ascent': 'ascent',
            'Descent': 'descent',
            'Touch Down': 'touchdown',
            // Backend telemetry modes
            'BOOT': 'pre-launch',
            'TEST_MODE': 'calibration',
            'LAUNCH_PAD': 'ready',
            'ASCENT': 'ascent',
            'ROCKET_DEPLOY': 'ascent',
            'DESCENT': 'descent',
            'AEROBRAKE_RELEASE': 'descent',
            'IMPACT': 'touchdown',
            'UNKNOWN': 'pre-launch'
        };

        // Remove active class from all phases
        document.querySelectorAll('.phase-item').forEach(item => {
            item.classList.remove('active');
        });

        // Add active class to current phase
        const currentPhase = phaseMapping[currentMode];
        if (currentPhase) {
            const phaseElement = document.querySelector(`[data-phase="${currentPhase}"]`);
            if (phaseElement) {
                phaseElement.classList.add('active');
            }
        } else {
            console.log(`Unknown mission phase: ${currentMode}`);
        }
    }

    applyStatusColors(data) {
        // Battery voltage status coloring (assuming typical LiPo 2S battery: 8.4V full, 6.0V empty)
        const batteryElement = document.getElementById('battery-voltage');
        if (batteryElement) {
            const voltage = data.battery_voltage || 0;
            batteryElement.className = 'data-value';
            if (voltage > 7.4) {
                batteryElement.classList.add('good');
            } else if (voltage > 6.6) {
                batteryElement.classList.add('warning');
            } else {
                batteryElement.classList.add('critical');
            }
        }

        // Temperature status coloring
        const tempElement = document.getElementById('temperature-value');
        if (tempElement) {
            const temp = data.temperature || 0;
            tempElement.className = 'data-value';
            if (temp > 50 || temp < 0) {
                tempElement.classList.add('critical');
            } else if (temp > 40 || temp < 5) {
                tempElement.classList.add('warning');
            } else {
                tempElement.classList.add('good');
            }
        }

        // Error indicator coloring
        const errorElement = document.getElementById('error-indicator');
        if (errorElement) {
            errorElement.className = 'data-value';
            if (data.error_indicator === 'All Good') {
                errorElement.classList.add('good');
            } else {
                errorElement.classList.add('critical');
            }
        }

        // GPS satellites coloring
        const gpsElement = document.getElementById('satellites-count');
        if (gpsElement) {
            const sats = data.num_satellites || 0;
            gpsElement.className = 'data-value';
            if (sats >= 6) {
                gpsElement.classList.add('good');
            } else if (sats >= 4) {
                gpsElement.classList.add('warning');
            } else {
                gpsElement.classList.add('critical');
            }
        }
    }

    addAlert(alert) {
        this.alerts.unshift(alert); // Add to beginning
        
        // Keep only last 20 alerts
        if (this.alerts.length > 20) {
            this.alerts = this.alerts.slice(0, 20);
        }

        this.updateAlertsDisplay();
    }

    updateAlertsDisplay() {
        const container = document.getElementById('alerts-container');
        if (!container) return;

        container.innerHTML = '';

        if (this.alerts.length === 0) {
            container.innerHTML = '<div class="alert-item info">No alerts</div>';
            return;
        }

        this.alerts.slice(0, 10).forEach(alert => { // Show only last 10 alerts
            const alertDiv = document.createElement('div');
            alertDiv.className = `alert-item ${alert.level}`;
            
            const time = new Date(alert.timestamp).toLocaleTimeString();
            alertDiv.innerHTML = `
                <div style="font-weight: bold; margin-bottom: 2px;">${time}</div>
                <div>${alert.message}</div>
            `;
            
            container.appendChild(alertDiv);
        });
    }

    togglePlotting() {
        this.isPlotting = !this.isPlotting;
        
        const button = document.getElementById('start-stop-btn');
        if (button) {
            if (this.isPlotting) {
                button.innerHTML = '🔴 STOP PLOTTING';
                button.style.background = '#f44336';
            } else {
                button.innerHTML = '🟢 START PLOTTING';
                button.style.background = '#00bcd4';
            }
        }

        // Start/stop mission via Eel
        if (this.isPlotting) {
            eel.start_mission()((result) => {
                if (result.status === 'success') {
                    this.addAlert({
                        timestamp: new Date().toISOString(),
                        level: 'success',
                        message: result.message
                    });
                    this.missionStartTime = new Date();
                }
            });
        } else {
            eel.stop_mission()((result) => {
                if (result.status === 'success') {
                    this.addAlert({
                        timestamp: new Date().toISOString(),
                        level: 'info',
                        message: result.message
                    });
                }
            });
        }

        return this.isPlotting;
    }

    resetDashboard() {
        // Clear charts
        if (window.chartsManager) {
            window.chartsManager.clearAllCharts();
        }

        // Clear alerts
        this.alerts = [];
        this.updateAlertsDisplay();

        // Reset mission data
        this.currentData = {};
        this.missionStartTime = null;

        // Reset UI elements to default values
        const defaultValues = {
            'mission-time': '0:00',
            'mission-time-sec': '0',
            'altitude-value': '0.00',
            'pressure-value': '0.00',
            'temperature-value': '0.0',
            'battery-voltage': '0.00',
            'speed-value': '0.00',
            'packet-count': '0',
            'satellites-count': '0',
            'error-indicator': 'All Good'
        };

        Object.entries(defaultValues).forEach(([id, value]) => {
            this.updateElement(id, value, false);
        });

        // Reset mission phase
        this.updateMissionPhase('Pre Launch');

        // Reset map
        if (window.mapManager) {
            window.mapManager.reset();
        }

        // Call backend reset
        eel.reset_dashboard()((result) => {
            this.addAlert({
                timestamp: new Date().toISOString(),
                level: 'success',
                message: result.message || 'Dashboard reset complete'
            });
        });
    }

    exportData(format) {
        eel.export_data(format)((result) => {
            if (result.status === 'success') {
                this.addAlert({
                    timestamp: new Date().toISOString(),
                    level: 'success',
                    message: `Data exported: ${result.filename}`
                });
            } else {
                this.addAlert({
                    timestamp: new Date().toISOString(),
                    level: 'error',
                    message: `Export failed: ${result.message}`
                });
            }
        });
    }

    calibrateSensors() {
        eel.calibrate_sensors()((result) => {
            const alertLevel = result.status === 'success' ? 'success' : 'error';
            this.addAlert({
                timestamp: new Date().toISOString(),
                level: alertLevel,
                message: result.message
            });
        });
    }

    showLoading(show = true) {
        const overlay = document.getElementById('loading-overlay');
        if (overlay) {
            overlay.style.display = show ? 'flex' : 'none';
        }
    }

    getMissionSummary() {
        return {
            startTime: this.missionStartTime,
            duration: this.missionStartTime ? Date.now() - this.missionStartTime.getTime() : 0,
            currentData: this.currentData,
            totalAlerts: this.alerts.length,
            isActive: this.isPlotting
        };
    }
}

// Make Dashboard available globally
window.Dashboard = Dashboard;