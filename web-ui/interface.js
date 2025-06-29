// Global state
let isKeyboardEnabled = false;
let isRobotEngaged = false;
let currentConfig = {};

// Network connection management
let bestServerUrl = null;
let bestWebSocketUrl = null;
let localIpDetected = null;

// Detect local network and find best connection method
async function detectBestConnection() {
  console.log('🔍 Detecting best connection method...');
  
  // If we're already on a local IP, use it directly
  const hostname = window.location.hostname;
  if (hostname.match(/^192\.168\./) || hostname.match(/^10\./) || hostname.match(/^172\.(1[6-9]|2[0-9]|3[0-1])\./) || hostname === 'localhost') {
    console.log('✅ Already on local network');
    bestServerUrl = window.location.origin;
    bestWebSocketUrl = `wss://${hostname}:8442`;
    return;
  }
  
  // Try to detect local IP by making test connections
  const commonLocalIPs = [
    '192.168.1.1', '192.168.1.100', '192.168.1.101', '192.168.1.200',
    '192.168.0.1', '192.168.0.100', '192.168.0.101', '192.168.0.200',
    '192.168.7.1', '192.168.7.100', '192.168.7.200', '192.168.7.233',
    '10.0.0.1', '10.0.0.100', '10.0.1.1', '10.0.1.100'
  ];
  
  // Test each IP for telegrip server
  for (const ip of commonLocalIPs) {
    try {
      console.log(`🔍 Testing connection to ${ip}:8443...`);
      
      // Create AbortController for timeout
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 2000); // 2 second timeout
      
      const response = await fetch(`https://${ip}:8443/api/status`, {
        method: 'GET',
        signal: controller.signal,
        mode: 'cors',
        cache: 'no-cache'
      });
      
      clearTimeout(timeoutId);
      
      if (response.ok) {
        console.log(`✅ Found telegrip server at ${ip}:8443`);
        localIpDetected = ip;
        bestServerUrl = `https://${ip}:8443`;
        bestWebSocketUrl = `wss://${ip}:8442`;
        return;
      }
    } catch (error) {
      // Connection failed, try next IP
      console.log(`❌ Failed to connect to ${ip}: ${error.message}`);
    }
  }
  
  // No local connection found, use tunnel
  console.log('🌐 No local connection found, using tunnel URL');
  bestServerUrl = window.location.origin;
  bestWebSocketUrl = `wss://${hostname}:8442`;
}

// Smart fetch that uses the best available connection
async function smartFetch(endpoint, options = {}) {
  if (!bestServerUrl) {
    await detectBestConnection();
  }
  
  const url = endpoint.startsWith('/') ? `${bestServerUrl}${endpoint}` : endpoint;
  
  try {
    return await fetch(url, options);
  } catch (error) {
    // If local connection fails, try tunnel as fallback
    if (bestServerUrl !== window.location.origin) {
      console.log('🔄 Local connection failed, falling back to tunnel');
      const fallbackUrl = endpoint.startsWith('/') ? `${window.location.origin}${endpoint}` : endpoint;
      return await fetch(fallbackUrl, options);
    }
    throw error;
  }
}

// Show connection status to user
function showConnectionStatus(message, type) {
  // Create status indicator if it doesn't exist
  let statusIndicator = document.getElementById('connectionStatus');
  if (!statusIndicator) {
    statusIndicator = document.createElement('div');
    statusIndicator.id = 'connectionStatus';
    statusIndicator.style.cssText = `
      position: fixed;
      top: 10px;
      right: 10px;
      padding: 8px 12px;
      border-radius: 6px;
      font-size: 12px;
      font-weight: bold;
      z-index: 1000;
      box-shadow: 0 2px 6px rgba(0,0,0,0.2);
    `;
    document.body.appendChild(statusIndicator);
  }
  
  // Set message and style based on connection type
  statusIndicator.textContent = message;
  if (type === 'local') {
    statusIndicator.style.backgroundColor = '#27ae60';
    statusIndicator.style.color = 'white';
  } else {
    statusIndicator.style.backgroundColor = '#f39c12';
    statusIndicator.style.color = 'white';
  }
  
  // Hide after 5 seconds
  setTimeout(() => {
    if (statusIndicator) {
      statusIndicator.style.opacity = '0.7';
    }
  }, 5000);
}

// Settings modal functions
function openSettings() {
  const modal = document.getElementById('settingsModal');
  modal.classList.add('show');
  loadConfiguration();
}

function closeSettings() {
  const modal = document.getElementById('settingsModal');
  modal.classList.remove('show');
}

function loadConfiguration() {
  smartFetch('/api/config')
    .then(response => response.json())
    .then(config => {
      currentConfig = config;
      populateSettingsForm(config);
    })
    .catch(error => {
      console.error('Error loading configuration:', error);
      alert('Error loading configuration');
    });
}

function populateSettingsForm(config) {
  // Robot arms
  document.getElementById('leftArmName').value = config.robot?.left_arm?.name || '';
  document.getElementById('leftArmPort').value = config.robot?.left_arm?.port || '';
  document.getElementById('rightArmName').value = config.robot?.right_arm?.name || '';
  document.getElementById('rightArmPort').value = config.robot?.right_arm?.port || '';
  
  // Network settings
  document.getElementById('httpsPort').value = config.network?.https_port || '';
  document.getElementById('websocketPort').value = config.network?.websocket_port || '';
  document.getElementById('hostIp').value = config.network?.host_ip || '';
  
  // Control parameters
  document.getElementById('vrScale').value = config.robot?.vr_to_robot_scale || '';
  document.getElementById('sendInterval').value = (config.robot?.send_interval * 1000) || ''; // Convert to ms
  document.getElementById('posStep').value = config.control?.keyboard?.pos_step || '';
  document.getElementById('angleStep').value = config.control?.keyboard?.angle_step || '';
}

function restartSystem() {
  if (!confirm('Are you sure you want to restart the system? This will temporarily disconnect all devices.')) {
    return;
  }

  const restartButton = document.getElementById('restartButton');
  restartButton.disabled = true;
  restartButton.textContent = 'Restarting...';

  smartFetch('/api/restart', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    }
  })
  .then(response => {
    if (response.ok) {
      // Show restart message and close modal
      alert('System is restarting... The page will reload automatically in a few seconds.');
      closeSettings();
      
      // Try to reconnect after a delay
      setTimeout(() => {
        window.location.reload();
      }, 5000);
    } else {
      alert('Failed to restart system. Please restart manually.');
    }
  })
  .catch(error => {
    console.error('Error restarting system:', error);
    alert('Error communicating with server. Please restart manually.');
  })
  .finally(() => {
    restartButton.disabled = false;
    restartButton.textContent = '🔄 Restart System';
  });
}

function saveConfiguration() {
  const form = document.getElementById('settingsForm');
  const formData = new FormData(form);
  
  // Build config object
  const updatedConfig = {
    robot: {
      left_arm: {
        name: formData.get('leftArmName'),
        port: formData.get('leftArmPort'),
        enabled: true
      },
      right_arm: {
        name: formData.get('rightArmName'),
        port: formData.get('rightArmPort'),
        enabled: true
      },
      vr_to_robot_scale: parseFloat(formData.get('vrScale')),
      send_interval: parseFloat(formData.get('sendInterval')) / 1000 // Convert from ms
    },
    network: {
      https_port: parseInt(formData.get('httpsPort')),
      websocket_port: parseInt(formData.get('websocketPort')),
      host_ip: formData.get('hostIp')
    },
    control: {
      keyboard: {
        pos_step: parseFloat(formData.get('posStep')),
        angle_step: parseFloat(formData.get('angleStep'))
      }
    }
  };

  const saveButton = document.getElementById('saveButton');
  saveButton.disabled = true;
  saveButton.textContent = 'Saving...';

  smartFetch('/api/config', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify(updatedConfig)
  })
  .then(response => response.json())
  .then(data => {
    if (data.success) {
      alert('Configuration saved successfully! Use the restart button to apply changes.');
    } else {
      alert('Failed to save configuration: ' + (data.error || 'Unknown error'));
    }
  })
  .catch(error => {
    console.error('Error saving configuration:', error);
    alert('Error saving configuration');
  })
  .finally(() => {
    saveButton.disabled = false;
    saveButton.textContent = '💾 Save Configuration';
  });
}

// Update status indicators
function updateStatus() {
  smartFetch('/api/status')
    .then(response => response.json())
    .then(data => {
      // Update arm connection indicators (based on device files)
      const leftIndicator = document.getElementById('leftArmStatus');
      const rightIndicator = document.getElementById('rightArmStatus');
      const vrIndicator = document.getElementById('vrStatus');
      
      leftIndicator.className = 'status-indicator' + (data.left_arm_connected ? ' connected' : '');
      rightIndicator.className = 'status-indicator' + (data.right_arm_connected ? ' connected' : '');
      vrIndicator.className = 'status-indicator' + (data.vrConnected ? ' connected' : '');
      
      // Update keyboard control status
      isKeyboardEnabled = data.keyboardEnabled;
      const keyboardHelp = document.querySelector('.keyboard-help');
      
      if (isKeyboardEnabled) {
        if (keyboardHelp) keyboardHelp.classList.add('active');
      } else {
        if (keyboardHelp) keyboardHelp.classList.remove('active');
      }
      
      // Update robot engagement status
      if (data.robotEngaged !== undefined) {
        isRobotEngaged = data.robotEngaged;
        updateEngagementUI();
      }
    })
    .catch(error => {
      console.error('Error fetching status:', error);
    });
}

function updateEngagementUI() {
  const engageBtn = document.getElementById('robotEngageBtn');
  const engageBtnText = document.getElementById('engageBtnText');
  const engagementStatusText = document.getElementById('engagementStatusText');
  
  if (isRobotEngaged) {
    engageBtn.classList.add('disconnect');
    engageBtnText.textContent = '🔌 Disconnect Robot';
    engagementStatusText.textContent = 'Motors Engaged';
    engagementStatusText.style.color = '#FFFFFF';
  } else {
    engageBtn.classList.remove('disconnect');
    engageBtnText.textContent = '🔌 Connect Robot';
    engagementStatusText.textContent = 'Motors Disengaged';
    engagementStatusText.style.color = '#FFFFFF';
  }
}

function toggleRobotEngagement() {
  const action = isRobotEngaged ? 'disconnect' : 'connect';
  
  smartFetch('/api/robot', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ action: action })
  })
  .then(response => response.json())
  .then(data => {
    if (data.success) {
      isRobotEngaged = !isRobotEngaged;
      updateEngagementUI();
    } else {
      alert('Failed to ' + action + ' robot: ' + (data.error || 'Unknown error'));
    }
  })
  .catch(error => {
    console.error('Error toggling robot engagement:', error);
    alert('Error communicating with server');
  });
}

// Toggle keyboard control
function toggleKeyboardControl() {
  const action = isKeyboardEnabled ? 'disable' : 'enable';
  
  smartFetch('/api/keyboard', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ action: action })
  })
  .then(response => response.json())
  .then(data => {
    if (data.success) {
      isKeyboardEnabled = !isKeyboardEnabled;
      const keyboardHelp = document.querySelector('.keyboard-help');
      
      if (isKeyboardEnabled) {
        if (keyboardHelp) keyboardHelp.classList.add('active');
      } else {
        if (keyboardHelp) keyboardHelp.classList.remove('active');
      }
    } else {
      alert('Failed to toggle keyboard control: ' + (data.error || 'Unknown error'));
    }
  })
  .catch(error => {
    console.error('Error toggling keyboard control:', error);
    alert('Error communicating with server');
  });
}

// Check if running in VR/AR mode
function isVRMode() {
  return window.navigator.xr && document.fullscreenElement;
}

// Update UI based on device
function updateUIForDevice() {
  const desktopInterface = document.getElementById('desktopInterface');
  const vrContent = document.getElementById('vrContent');
  
  if (isVRMode()) {
    desktopInterface.style.display = 'none';
    vrContent.style.display = 'none';
  } else {
    // Check if this is a VR-capable device
    if (navigator.xr) {
      navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
        if (supported) {
          // VR-capable device - show VR interface
          desktopInterface.style.display = 'none';
          vrContent.style.display = 'block';
        } else {
          // Not VR-capable - show desktop interface
          desktopInterface.style.display = 'block';
          vrContent.style.display = 'none';
        }
      }).catch(() => {
        // Fallback to desktop interface if XR check fails
        desktopInterface.style.display = 'block';
        vrContent.style.display = 'none';
      });
    } else {
      // No XR support - show desktop interface
      desktopInterface.style.display = 'block';
      vrContent.style.display = 'none';
    }
  }
}

// Web-based keyboard control
let pressedKeys = new Set();

// Add keyboard event listeners for web-based control
document.addEventListener('keydown', handleKeyDown);
document.addEventListener('keyup', handleKeyUp);

function handleKeyDown(event) {
  // Prevent default browser behavior for our control keys regardless of keyboard state
  if (isControlKey(event.code)) {
    event.preventDefault();
  }
  
  // Only handle keys if keyboard control is enabled and we're focused on the page
  if (!isKeyboardEnabled || pressedKeys.has(event.code)) return;
  
  if (isControlKey(event.code)) {
    pressedKeys.add(event.code);
    sendKeyCommand(event.code, 'press');
  }
}

function handleKeyUp(event) {
  // Prevent default browser behavior for our control keys regardless of keyboard state
  if (isControlKey(event.code)) {
    event.preventDefault();
  }
  
  // Only handle keys if keyboard control is enabled
  if (!isKeyboardEnabled || !pressedKeys.has(event.code)) return;
  
  if (isControlKey(event.code)) {
    pressedKeys.delete(event.code);
    sendKeyCommand(event.code, 'release');
  }
}

function isControlKey(code) {
  // Check if this is one of our robot control keys
  const controlKeys = [
    // Left arm
    'KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE', 
    'KeyZ', 'KeyX', 'KeyF',
    // Right arm
    'KeyI', 'KeyK', 'KeyJ', 'KeyL', 'KeyU', 'KeyO',
    'KeyN', 'KeyM', 'Semicolon',
    // Global
    'Escape'
  ];
  return controlKeys.includes(code);
}

function sendKeyCommand(keyCode, action) {
  // Convert browser keyCode to our key mapping
  const keyMap = {
    // Left arm
    'KeyW': 'w', 'KeyS': 's', 'KeyA': 'a', 'KeyD': 'd',
    'KeyQ': 'q', 'KeyE': 'e', 'KeyZ': 'z', 'KeyX': 'x',
    'KeyF': 'f',
    // Right arm  
    'KeyI': 'i', 'KeyK': 'k', 'KeyJ': 'j', 'KeyL': 'l',
    'KeyU': 'u', 'KeyO': 'o', 'KeyN': 'n', 'KeyM': 'm',
    'Semicolon': ';',
    // Global
    'Escape': 'esc'
  };

  const key = keyMap[keyCode];
  if (!key) return;

  smartFetch('/api/keypress', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ 
      key: key, 
      action: action 
    })
  })
  .catch(error => {
    console.error('Error sending key command:', error);
  });
}

// Initialize
document.addEventListener('DOMContentLoaded', async () => {
  updateUIForDevice();
  
  // Detect best connection method
  await detectBestConnection();
  
  // Expose best WebSocket URL globally for VR app
  window.bestWebSocketUrl = bestWebSocketUrl;
  
  // Show connection status to user
  if (localIpDetected) {
    console.log(`🚀 Using local network connection: ${localIpDetected} (Ultra-low latency)`);
    // Add visual indicator for local connection
    showConnectionStatus(`🚀 Local Network (${localIpDetected})`, 'local');
  } else {
    console.log(`🌐 Using tunnel connection (Normal latency)`);
    // Add visual indicator for tunnel connection
    showConnectionStatus('🌐 Tunnel Connection', 'tunnel');
  }
  
  // Start status monitoring
  updateStatus();
  setInterval(updateStatus, 2000); // Update every 2 seconds
  
  // Handle VR mode changes
  document.addEventListener('fullscreenchange', updateUIForDevice);
  
  // VR session detection
  if (navigator.xr) {
    navigator.xr.addEventListener('sessionstart', () => {
      updateStatus();
      updateUIForDevice();
    });
    
    navigator.xr.addEventListener('sessionend', () => {
      updateStatus();
      updateUIForDevice();
    });
  }

  // Settings form handler
  document.getElementById('settingsForm').addEventListener('submit', (e) => {
    e.preventDefault();
    saveConfiguration();
  });

  // Close modal when clicking outside
  document.getElementById('settingsModal').addEventListener('click', (e) => {
    if (e.target.id === 'settingsModal') {
      closeSettings();
    }
  });
});

// Handle window resize
window.addEventListener('resize', updateUIForDevice); 