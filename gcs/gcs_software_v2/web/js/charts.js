/**
 * Charts Module - Handles all chart creation and updates
 */

class ChartsManager {
    constructor() {
        this.charts = {};
        this.maxDataPoints = 50;
        this.chartColors = {
            primary: '#00bcd4',
            secondary: '#4fc3f7',
            success: '#4caf50',
            warning: '#ff9800',
            error: '#f44336',
            x_axis: '#f44336',
            y_axis: '#ffeb3b',
            z_axis: '#2196f3'
        };
        this.initializeCharts();
    }

    initializeCharts() {
        // Set Chart.js global defaults
        Chart.defaults.color = '#ffffff';
        Chart.defaults.borderColor = 'rgba(255, 255, 255, 0.1)';
        
        this.createReactionWheelChart();
        this.createGyroscopeChart();
        this.createAltitudeChart();
        this.createPressureChart();
        this.createTemperatureChart();
        this.createAccelerometerChart();
    }

    createReactionWheelChart() {
        const ctx = document.getElementById('reactionWheelChart').getContext('2d');
        
        // Generate initial data points for reaction wheel
        const initialData = Array.from({length: 20}, () => Math.floor(Math.random() * 80) + 10);
        
        this.charts.reactionWheel = new Chart(ctx, {
            type: 'line',
            data: {
                labels: Array.from({length: 20}, (_, i) => i),
                datasets: [{
                    label: 'Reaction Wheel',
                    data: initialData,
                    borderColor: this.chartColors.primary,
                    backgroundColor: 'rgba(0, 188, 212, 0.1)',
                    borderWidth: 2,
                    fill: true,
                    tension: 0.4,
                    pointRadius: 2,
                    pointHoverRadius: 4
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        display: false
                    }
                },
                scales: {
                    x: {
                        display: false,
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        }
                    },
                    y: {
                        min: 0,
                        max: 100,
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#ffffff',
                            font: {
                                size: 10
                            }
                        }
                    }
                },
                interaction: {
                    intersect: false
                }
            }
        });
    }

    createGyroscopeChart() {
        const ctx = document.getElementById('gyroscopeChart').getContext('2d');
        
        this.charts.gyroscope = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'X',
                        data: [],
                        borderColor: this.chartColors.x_axis,
                        backgroundColor: 'rgba(244, 67, 54, 0.1)',
                        borderWidth: 2,
                        fill: false,
                        tension: 0.4,
                        pointRadius: 1
                    },
                    {
                        label: 'Y',
                        data: [],
                        borderColor: this.chartColors.y_axis,
                        backgroundColor: 'rgba(255, 235, 59, 0.1)',
                        borderWidth: 2,
                        fill: false,
                        tension: 0.4,
                        pointRadius: 1
                    },
                    {
                        label: 'Z',
                        data: [],
                        borderColor: this.chartColors.z_axis,
                        backgroundColor: 'rgba(33, 150, 243, 0.1)',
                        borderWidth: 2,
                        fill: false,
                        tension: 0.4,
                        pointRadius: 1
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        display: false
                    }
                },
                scales: {
                    x: {
                        display: false,
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        }
                    },
                    y: {
                        min: -3,
                        max: 3,
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#ffffff',
                            font: {
                                size: 10
                            }
                        }
                    }
                }
            }
        });
    }

    createAltitudeChart() {
        const ctx = document.getElementById('altitudeChart').getContext('2d');
        
        this.charts.altitude = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Altitude',
                    data: [],
                    borderColor: this.chartColors.primary,
                    backgroundColor: 'rgba(0, 188, 212, 0.2)',
                    borderWidth: 2,
                    fill: true,
                    tension: 0.4,
                    pointRadius: 1
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        display: false
                    }
                },
                scales: {
                    x: {
                        display: false
                    },
                    y: {
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#ffffff',
                            font: {
                                size: 9
                            }
                        }
                    }
                }
            }
        });
    }

    createPressureChart() {
        const ctx = document.getElementById('pressureChart').getContext('2d');
        
        this.charts.pressure = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Pressure',
                    data: [],
                    borderColor: this.chartColors.secondary,
                    backgroundColor: 'rgba(79, 195, 247, 0.2)',
                    borderWidth: 2,
                    fill: true,
                    tension: 0.4,
                    pointRadius: 1
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        display: false
                    }
                },
                scales: {
                    x: {
                        display: false
                    },
                    y: {
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#ffffff',
                            font: {
                                size: 9
                            }
                        }
                    }
                }
            }
        });
    }

    createTemperatureChart() {
        const ctx = document.getElementById('temperatureChart').getContext('2d');
        
        this.charts.temperature = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Temperature',
                    data: [],
                    borderColor: this.chartColors.warning,
                    backgroundColor: 'rgba(255, 152, 0, 0.2)',
                    borderWidth: 2,
                    fill: true,
                    tension: 0.4,
                    pointRadius: 1
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        display: false
                    }
                },
                scales: {
                    x: {
                        display: false
                    },
                    y: {
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#ffffff',
                            font: {
                                size: 9
                            }
                        }
                    }
                }
            }
        });
    }

    createAccelerometerChart() {
        const ctx = document.getElementById('accelerometerChart').getContext('2d');
        
        this.charts.accelerometer = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'X',
                        data: [],
                        borderColor: this.chartColors.x_axis,
                        backgroundColor: 'rgba(244, 67, 54, 0.1)',
                        borderWidth: 1.5,
                        fill: false,
                        tension: 0.4,
                        pointRadius: 0.5
                    },
                    {
                        label: 'Y',
                        data: [],
                        borderColor: this.chartColors.y_axis,
                        backgroundColor: 'rgba(255, 235, 59, 0.1)',
                        borderWidth: 1.5,
                        fill: false,
                        tension: 0.4,
                        pointRadius: 0.5
                    },
                    {
                        label: 'Z',
                        data: [],
                        borderColor: this.chartColors.z_axis,
                        backgroundColor: 'rgba(33, 150, 243, 0.1)',
                        borderWidth: 1.5,
                        fill: false,
                        tension: 0.4,
                        pointRadius: 0.5
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        display: false
                    }
                },
                scales: {
                    x: {
                        display: false
                    },
                    y: {
                        min: -2,
                        max: 12,
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        },
                        ticks: {
                            color: '#ffffff',
                            font: {
                                size: 9
                            }
                        }
                    }
                }
            }
        });
    }

    updateChart(chartName, data, timestamp) {
        const chart = this.charts[chartName];
        if (!chart) return;

        const timeLabel = new Date(timestamp).toLocaleTimeString();

        if (chartName === 'reactionWheel') {
            // Update reaction wheel with array data
            chart.data.datasets[0].data = data.reaction_wheel || [];
            chart.data.labels = Array.from({length: data.reaction_wheel?.length || 0}, (_, i) => i);
        } else if (chartName === 'gyroscope') {
            // Update gyroscope with x, y, z data
            this.addDataPoint(chart, timeLabel, [
                data.gyroscope?.x || 0,
                data.gyroscope?.y || 0,
                data.gyroscope?.z || 0
            ]);
        } else if (chartName === 'accelerometer') {
            // Update accelerometer with x, y, z data
            this.addDataPoint(chart, timeLabel, [
                data.accelerometer?.x || 0,
                data.accelerometer?.y || 0,
                data.accelerometer?.z || 0
            ]);
        } else {
            // Single value charts
            const value = data[chartName] || 0;
            this.addDataPoint(chart, timeLabel, [value]);
        }

        chart.update('none'); // Use 'none' animation for better performance
    }

    addDataPoint(chart, label, values) {
        chart.data.labels.push(label);
        
        values.forEach((value, index) => {
            if (chart.data.datasets[index]) {
                chart.data.datasets[index].data.push(value);
            }
        });

        // Keep only last maxDataPoints
        if (chart.data.labels.length > this.maxDataPoints) {
            chart.data.labels.shift();
            chart.data.datasets.forEach(dataset => {
                dataset.data.shift();
            });
        }
    }

    resizeCharts() {
        Object.values(this.charts).forEach(chart => {
            chart.resize();
        });
    }

    clearAllCharts() {
        Object.values(this.charts).forEach(chart => {
            chart.data.labels = [];
            chart.data.datasets.forEach(dataset => {
                dataset.data = [];
            });
            chart.update();
        });
    }

    destroyCharts() {
        Object.values(this.charts).forEach(chart => {
            chart.destroy();
        });
        this.charts = {};
    }
}

// Make ChartsManager available globally
window.ChartsManager = ChartsManager;