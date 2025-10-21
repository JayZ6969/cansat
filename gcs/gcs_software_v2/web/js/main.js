/**
 * Main Application JavaScript
 * Coordinates all modules and handles Eel communication
 */

// Global variables
let chartsManager = null;
let dashboard = null;
let mapManager = null;
let dataUpdateInterval = null;
let isApplicationReady = false;
let isConnected = false;
let availablePorts = [];

// Initialize application when DOM is loaded
document.addEventListener('DOMContentLoaded', function() {
    initializeApplication();
});

function initializeApplication() {
    console.log('Initializing GCS Application...');
    
    try {
        // Initialize modules
        chartsManager = new ChartsManager();
        dashboard = new Dashboard();
        mapManager = new MapManager();
        
        // Make modules globally accessible
        window.chartsManager = chartsManager;
        window.dashboard = dashboard;
        window.mapManager = mapManager;
        
        // Setup Eel communication
        setupEelCommunication();
        
        // Initialize COM port functionality
        initializeComPortControls();
        
        // Start data update loop
        startDataUpdateLoop();
        
        // Hide loading overlay
        dashboard.showLoading(false);
        
        isApplicationReady = true;
        console.log('GCS Application initialized successfully');
        
        // Add welcome alert
        dashboard.addAlert({
            timestamp: new Date().toISOString(),
            level: 'info',
            message: 'Ground Control Station ready - Scan for COM ports to connect'
        });
        
        // Automatically scan for COM ports
        scanComPorts();
        
    } catch (error) {
        console.error('Failed to initialize application:', error);
        showInitializationError(error);
    }
}

function setupEelCommunication() {
    // Check if Eel is available
    if (typeof eel === 'undefined') {
        console.error('Eel is not available - running in demo mode');
        startDemoMode();
        return;
    }
    
    // Setup Eel exposed functions that can be called from Python
    eel.expose(update_telemetry);
    eel.expose(add_alert);
    eel.expose(update_mission_status);
    eel.expose(update_dashboard);
    
    console.log('Eel communication setup complete');
}

// Function called from Python to update telemetry data
function update_telemetry(data) {
    if (!isApplicationReady || !dashboard) {
        return;
    }
    
    try {
        dashboard.updateTelemetryDisplay(data);
    } catch (error) {
        console.error('Error updating telemetry:', error);
    }
}

// Alias for update_telemetry (called from Python backend)
function update_dashboard(data) {
    update_telemetry(data);
}

// Function called from Python to add alerts
function add_alert(alert) {
    if (!isApplicationReady || !dashboard) {
        return;
    }
    
    try {
        dashboard.addAlert(alert);
    } catch (error) {
        console.error('Error adding alert:', error);
    }
}

// Function called from Python to update mission status
function update_mission_status(status) {
    if (!isApplicationReady || !dashboard) {
        return;
    }
    
    try {
        // Update mission phase and other status indicators
        dashboard.updateMissionPhase(status.current_phase);
        
        // Add any new alerts
        if (status.alerts && status.alerts.length > 0) {
            status.alerts.forEach(alert => {
                dashboard.addAlert(alert);
            });
        }
    } catch (error) {
        console.error('Error updating mission status:', error);
    }
}

function startDataUpdateLoop() {
    // Data is pushed from Python via eel.update_dashboard()
    // No polling needed - the backend pushes data when available
    console.log('Data update loop: Using push-based updates from backend');
}

function startDemoMode() {
    console.log('Starting demo mode with simulated data...');
    
    // Create demo telemetry handler for testing without backend
    const demoInterval = setInterval(() => {
        if (!dashboard || !dashboard.isPlotting) {
            return;
        }
        
        const demoData = generateDemoData();
        update_telemetry(demoData);
    }, 500); // Update every 500ms in demo mode
    
    dashboard.addAlert({
        timestamp: new Date().toISOString(),
        level: 'warning',
        message: 'Running in demo mode - No backend connection'
    });
}

function generateDemoData() {
    const time = Date.now() / 1000;
    const missionTime = Math.floor(time % 1000);
    
    // Simulate mission phases
    const phases = ['Pre Launch', 'Calibration', 'Ready to Launch', 'Ascent', 'Descent', 'Touch Down'];
    const phaseIndex = Math.floor(time / 30) % phases.length;
    
    return {
        timestamp: new Date().toISOString(),
        mission_time: missionTime,
        altitude: Math.max(0, 939.86 + Math.random() * 100 - 50 + 20 * Math.sin(time * 0.01)),
        pressure: 1013.25 + Math.random() * 20 - 10,
        temperature: 25.0 + Math.random() * 10 - 5,
        battery_percentage: Math.max(0, Math.min(100, 95 + Math.random() * 10 - 8)),
        battery_voltage: 8.3 + Math.random() * 0.5 - 0.3,
        speed: Math.random() * 3,
        packet_count: missionTime,
        gps_latitude: 40.7128 + (Math.random() - 0.5) * 0.01,
        gps_longitude: -74.0060 + (Math.random() - 0.5) * 0.01,
        gyroscope: {
            x: (Math.random() - 0.5) * 4,
            y: (Math.random() - 0.5) * 4,
            z: (Math.random() - 0.5) * 4
        },
        accelerometer: {
            x: (Math.random() - 0.5) * 2,
            y: (Math.random() - 0.5) * 2,
            z: 9.8 + (Math.random() - 0.5) * 2
        },
        reaction_wheel: (Math.random() - 0.5) * 400,  // PID output value (-200 to 200)
        mode: phases[phaseIndex],
        calibration_done: phaseIndex >= 1,
        parachute_deployed: phaseIndex >= 4,
        camera_recording: phaseIndex >= 2 && phaseIndex <= 4,
        num_satellites: Math.floor(Math.random() * 8) + 6,
        error_indicator: Math.random() > 0.9 ? 'Warning' : 'All Good'
    };
}

// Global functions called by HTML buttons
function toggleMission() {
    if (!dashboard) {
        console.error('Dashboard not initialized');
        return;
    }
    
    // Only allow mission toggle if connected to serial port
    if (!isConnected) {
        dashboard.addAlert({
            timestamp: new Date().toISOString(),
            level: 'warning',
            message: 'Please connect to a COM port before starting mission'
        });
        return;
    }
    
    // Connected mode - toggle the mission
    dashboard.togglePlotting();
}

function toggleTelemetry() {
    // Toggle telemetry panel visibility or functionality
    dashboard.addAlert({
        timestamp: new Date().toISOString(),
        level: 'info',
        message: 'Telemetry view toggled'
    });
}

function calibrateSensors() {
    if (!dashboard) {
        console.error('Dashboard not initialized');
        return;
    }
    
    dashboard.calibrateSensors();
}

function resetDashboard() {
    if (!dashboard) {
        console.error('Dashboard not initialized');
        return;
    }
    
    dashboard.resetDashboard();
}

function exportData(format) {
    if (!dashboard) {
        console.error('Dashboard not initialized');
        return;
    }
    
    dashboard.exportData(format);
}

// COM Port Management Functions
function initializeComPortControls() {
    console.log('Initializing COM port controls...');
    
    // Set default baud rate
    const baudSelect = document.getElementById('baud-rate-select');
    if (baudSelect) {
        baudSelect.value = '9600';
    }
    
    // Update connection status
    updateConnectionStatus(false);
}

function scanComPorts() {
    console.log('Scanning for COM ports...');
    
    dashboard.addAlert({
        timestamp: new Date().toISOString(),
        level: 'info',
        message: 'Scanning for available COM ports...'
    });
    
    if (typeof eel !== 'undefined') {
        eel.get_available_ports()((ports) => {
            availablePorts = ports || [];
            updateComPortDropdown();
            
            if (availablePorts.length === 0) {
                dashboard.addAlert({
                    timestamp: new Date().toISOString(),
                    level: 'warning',
                    message: 'No COM ports found. Check device connections.'
                });
            } else {
                dashboard.addAlert({
                    timestamp: new Date().toISOString(),
                    level: 'success',
                    message: `Found ${availablePorts.length} COM port(s)`
                });
            }
        });
    } else {
        // Demo mode - simulate some COM ports
        availablePorts = ['COM3', 'COM4', 'COM5'];
        updateComPortDropdown();
        dashboard.addAlert({
            timestamp: new Date().toISOString(),
            level: 'info',
            message: 'Demo mode: Simulated COM ports available'
        });
    }
}

function updateComPortDropdown() {
    const dropdown = document.getElementById('com-port-select');
    if (!dropdown) return;
    
    // Clear existing options except the first one
    dropdown.innerHTML = '<option value="">Select COM Port...</option>';
    
    // Add available ports
    availablePorts.forEach(port => {
        const option = document.createElement('option');
        
        // Check if port is an object or a string
        if (typeof port === 'object' && port.port) {
            // Port is an object with port, description, and hwid
            option.value = port.port;
            option.textContent = `${port.port} - ${port.description}`;
        } else {
            // Port is a simple string
            option.value = port;
            option.textContent = port;
        }
        
        dropdown.appendChild(option);
    });
}

function toggleConnection() {
    if (isConnected) {
        disconnectFromPort();
    } else {
        connectToPort();
    }
}

function connectToPort() {
    const portSelect = document.getElementById('com-port-select');
    const baudSelect = document.getElementById('baud-rate-select');
    
    if (!portSelect || !baudSelect) {
        console.error('COM port controls not found');
        return;
    }
    
    const selectedPort = portSelect.value;
    const selectedBaud = parseInt(baudSelect.value);
    
    if (!selectedPort) {
        dashboard.addAlert({
            timestamp: new Date().toISOString(),
            level: 'warning',
            message: 'Please select a COM port first'
        });
        return;
    }
    
    console.log(`Connecting to ${selectedPort} at ${selectedBaud} baud...`);
    
    // Update UI to show connecting state
    updateConnectionStatus('connecting');
    
    dashboard.addAlert({
        timestamp: new Date().toISOString(),
        level: 'info',
        message: `Connecting to ${selectedPort} at ${selectedBaud} baud...`
    });
    
    if (typeof eel !== 'undefined') {
        eel.connect_to_port(selectedPort, selectedBaud)((result) => {
            if (result.status === 'success') {
                isConnected = true;
                updateConnectionStatus(true);
                
                dashboard.addAlert({
                    timestamp: new Date().toISOString(),
                    level: 'success',
                    message: `Connected to ${selectedPort}`
                });
                
                // Enable mission controls
                enableMissionControls(true);
                
            } else {
                updateConnectionStatus(false);
                dashboard.addAlert({
                    timestamp: new Date().toISOString(),
                    level: 'error',
                    message: `Connection failed: ${result.message}`
                });
            }
        });
    } else {
        // Demo mode - simulate successful connection
        setTimeout(() => {
            isConnected = true;
            updateConnectionStatus(true);
            
            dashboard.addAlert({
                timestamp: new Date().toISOString(),
                level: 'success',
                message: `Demo: Connected to ${selectedPort}`
            });
            
            enableMissionControls(true);
        }, 1000);
    }
}

function disconnectFromPort() {
    console.log('Disconnecting from COM port...');
    
    dashboard.addAlert({
        timestamp: new Date().toISOString(),
        level: 'info',
        message: 'Disconnecting...'
    });
    
    if (typeof eel !== 'undefined') {
        eel.disconnect_from_port()((result) => {
            isConnected = false;
            updateConnectionStatus(false);
            
            dashboard.addAlert({
                timestamp: new Date().toISOString(),
                level: 'info',
                message: 'Disconnected from COM port'
            });
            
            // Disable mission controls
            enableMissionControls(false);
            
            // Stop mission if running
            if (dashboard.isPlotting) {
                dashboard.togglePlotting();
            }
        });
    } else {
        // Demo mode
        isConnected = false;
        updateConnectionStatus(false);
        enableMissionControls(false);
        
        dashboard.addAlert({
            timestamp: new Date().toISOString(),
            level: 'info',
            message: 'Demo: Disconnected from COM port'
        });
    }
}

function updateConnectionStatus(status) {
    const connectBtn = document.getElementById('connect-btn');
    
    console.log('updateConnectionStatus called with status:', status);
    console.log('Button element:', connectBtn);
    
    if (!connectBtn) {
        console.error('Connect button not found!');
        return;
    }
    
    // Remove all status classes
    connectBtn.classList.remove('connected', 'connecting');
    
    if (status === 'connecting') {
        console.log('Setting button to CONNECTING state');
        connectBtn.classList.add('connecting');
        connectBtn.innerHTML = '<span class="btn-icon">⏳</span><span class="btn-text">CONNECTING</span>';
        connectBtn.disabled = true;
    } else if (status === true) {
        console.log('Setting button to CONNECTED (DISCONNECT) state');
        connectBtn.classList.add('connected');
        connectBtn.innerHTML = '<span class="btn-icon">🔌</span><span class="btn-text">DISCONNECT</span>';
        connectBtn.disabled = false;
        isConnected = true;
        console.log('Button classes after connected:', connectBtn.className);
    } else {
        console.log('Setting button to DISCONNECTED (CONNECT) state');
        connectBtn.innerHTML = '<span class="btn-icon">📡</span><span class="btn-text">CONNECT</span>';
        connectBtn.disabled = false;
        isConnected = false;
        console.log('Button classes after disconnected:', connectBtn.className);
    }
}

function enableMissionControls(enable) {
    const startStopBtn = document.getElementById('start-stop-btn');
    
    if (startStopBtn) {
        startStopBtn.disabled = !enable;
        if (enable) {
            startStopBtn.style.opacity = '1';
            startStopBtn.style.cursor = 'pointer';
        } else {
            startStopBtn.style.opacity = '0.5';
            startStopBtn.style.cursor = 'not-allowed';
        }
    }
}

// Keyboard shortcuts
document.addEventListener('keydown', function(event) {
    if (!isApplicationReady) return;
    
    // Space bar to toggle mission
    if (event.code === 'Space' && !event.target.matches('input, textarea')) {
        event.preventDefault();
        toggleMission();
    }
    
    // R key to reset dashboard
    if (event.code === 'KeyR' && event.ctrlKey) {
        event.preventDefault();
        resetDashboard();
    }
    
    // C key to calibrate
    if (event.code === 'KeyC' && event.ctrlKey) {
        event.preventDefault();
        calibrateSensors();
    }
    
    // E key to export CSV
    if (event.code === 'KeyE' && event.ctrlKey) {
        event.preventDefault();
        exportData('csv');
    }
});

// Handle window resize
window.addEventListener('resize', function() {
    if (chartsManager) {
        chartsManager.resizeCharts();
    }
    if (mapManager) {
        mapManager.resize();
    }
});

// Handle window beforeunload
window.addEventListener('beforeunload', function() {
    if (dataUpdateInterval) {
        clearInterval(dataUpdateInterval);
    }
    
    if (typeof eel !== 'undefined' && dashboard && dashboard.isPlotting) {
        // Stop mission before closing
        eel.stop_mission();
    }
});

// Error handling
window.addEventListener('error', function(event) {
    console.error('Application error:', event.error);
    
    if (dashboard) {
        dashboard.addAlert({
            timestamp: new Date().toISOString(),
            level: 'error',
            message: `Application error: ${event.error.message}`
        });
    }
});

function showInitializationError(error) {
    const container = document.body;
    container.innerHTML = `
        <div style="
            display: flex;
            align-items: center;
            justify-content: center;
            height: 100vh;
            background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
            color: white;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            text-align: center;
            padding: 20px;
        ">
            <div>
                <div style="font-size: 48px; margin-bottom: 20px;">⚠️</div>
                <h2>Application Initialization Failed</h2>
                <p style="margin: 20px 0; color: #ff9800;">${error.message}</p>
                <button onclick="location.reload()" style="
                    padding: 10px 20px;
                    background: #00bcd4;
                    color: white;
                    border: none;
                    border-radius: 4px;
                    cursor: pointer;
                    font-size: 16px;
                ">Reload Application</button>
            </div>
        </div>
    `;
}

// Application status indicator
function updateConnectionStatus(connected) {
    const statusElements = document.querySelectorAll('.connection-status');
    statusElements.forEach(element => {
        element.style.background = connected ? '#4caf50' : '#f44336';
        element.title = connected ? 'Connected to CanSat' : 'Disconnected';
    });
}

// Export functions for external use
window.GCSApp = {
    initialize: initializeApplication,
    toggleMission: toggleMission,
    resetDashboard: resetDashboard,
    exportData: exportData,
    calibrateSensors: calibrateSensors,
    updateConnectionStatus: updateConnectionStatus
};

console.log('Main application script loaded');