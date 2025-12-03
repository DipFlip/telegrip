// Global state
let isKeyboardEnabled = false;
let isRobotEngaged = false;
let currentConfig = {};
let warningTimeout = null;

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
  fetch('/api/config')
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

  fetch('/api/restart', {
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

  fetch('/api/config', {
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
  fetch('/api/status')
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
  const connectionHint = document.getElementById('connectionHint');
  const connectionWarning = document.getElementById('connectionWarning');

  if (isRobotEngaged) {
    engageBtn.classList.add('disconnect');
    engageBtn.classList.remove('needs-attention');
    engageBtnText.textContent = '🔌 Disconnect Robot';
    engagementStatusText.textContent = 'Motors Engaged';
    engagementStatusText.style.color = '#FFFFFF';
    if (connectionHint) connectionHint.style.display = 'none';
    if (connectionWarning) connectionWarning.classList.remove('show');
  } else {
    engageBtn.classList.remove('disconnect');
    engageBtnText.textContent = '🔌 Connect Robot';
    engagementStatusText.textContent = 'Motors Disengaged';
    engagementStatusText.style.color = '#FFFFFF';
    if (connectionHint) connectionHint.style.display = 'block';
  }
}

function showConnectionWarning() {
  const engageBtn = document.getElementById('robotEngageBtn');
  const connectionWarning = document.getElementById('connectionWarning');

  if (!isRobotEngaged) {
    // Add pulsing animation to the connect button
    engageBtn.classList.add('needs-attention');

    // Show the warning message
    if (connectionWarning) {
      connectionWarning.classList.add('show');
    }

    // Clear any existing timeout
    if (warningTimeout) {
      clearTimeout(warningTimeout);
    }

    // Hide warning and stop pulsing after 5 seconds
    warningTimeout = setTimeout(() => {
      engageBtn.classList.remove('needs-attention');
      if (connectionWarning) {
        connectionWarning.classList.remove('show');
      }
    }, 5000);
  }
}

function toggleRobotEngagement() {
  const action = isRobotEngaged ? 'disconnect' : 'connect';
  
  fetch('/api/robot', {
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
  
  fetch('/api/keyboard', {
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

  if (isVRMode()) {
    desktopInterface.style.display = 'none';
  } else {
    // Check if this is a VR-capable device
    if (navigator.xr) {
      navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
        if (supported) {
          // VR-capable device - hide desktop interface (custom button in vr_app.js handles VR)
          desktopInterface.style.display = 'none';
        } else {
          // Not VR-capable - show desktop interface
          desktopInterface.style.display = 'block';
        }
      }).catch(() => {
        // Fallback to desktop interface if XR check fails
        desktopInterface.style.display = 'block';
      });
    } else {
      // No XR support - show desktop interface
      desktopInterface.style.display = 'block';
    }
  }
}

// Web-based keyboard control
let pressedKeys = new Set();

// Add keyboard event listeners for web-based control
// Use capture phase to intercept keys before browser handles them (e.g., F for fullscreen)
document.addEventListener('keydown', handleKeyDown, { capture: true });
document.addEventListener('keyup', handleKeyUp, { capture: true });

function handleKeyDown(event) {
  // Prevent default browser behavior for our control keys regardless of keyboard state
  if (isControlKey(event.code)) {
    event.preventDefault();
  }

  // Show warning if robot is not connected and user presses control keys
  if (isControlKey(event.code) && !isRobotEngaged) {
    showConnectionWarning();
    return;
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
    // Left arm movement
    'KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE',
    // Left arm wrist
    'KeyZ', 'KeyX',  // wrist roll
    'KeyR', 'KeyT',  // wrist flex
    'KeyF',          // gripper
    'Tab',           // toggle position control
    // Right arm movement
    'KeyI', 'KeyK', 'KeyJ', 'KeyL', 'KeyU', 'KeyO',
    // Right arm wrist
    'KeyN', 'KeyM',  // wrist roll
    'KeyH', 'KeyY',  // wrist flex
    'Semicolon',     // gripper
    'Enter',         // toggle position control
    // Global
    'Escape'
  ];
  return controlKeys.includes(code);
}

function sendKeyCommand(keyCode, action) {
  // Convert browser keyCode to our key mapping
  const keyMap = {
    // Left arm movement
    'KeyW': 'w', 'KeyS': 's', 'KeyA': 'a', 'KeyD': 'd',
    'KeyQ': 'q', 'KeyE': 'e',
    // Left arm wrist
    'KeyZ': 'z', 'KeyX': 'x',  // wrist roll
    'KeyR': 'r', 'KeyT': 't',  // wrist flex
    'KeyF': 'f',               // gripper
    'Tab': 'tab',              // toggle position control
    // Right arm movement
    'KeyI': 'i', 'KeyK': 'k', 'KeyJ': 'j', 'KeyL': 'l',
    'KeyU': 'u', 'KeyO': 'o',
    // Right arm wrist
    'KeyN': 'n', 'KeyM': 'm',  // wrist roll
    'KeyH': 'h', 'KeyY': 'y',  // wrist flex
    'Semicolon': ';',          // gripper
    'Enter': 'enter',          // toggle position control
    // Global
    'Escape': 'esc'
  };

  const key = keyMap[keyCode];
  if (!key) return;

  fetch('/api/keypress', {
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
document.addEventListener('DOMContentLoaded', () => {
  updateUIForDevice();
  
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

// Switch between desktop and VR views
function switchToVrView() {
  const desktopInterface = document.getElementById('desktopInterface');
  desktopInterface.style.display = 'none';
  // The VR start button should already be visible or will be created by vr_app.js
  // Trigger the button creation if not already done
  if (!document.getElementById('start-tracking-button')) {
    // Create a simple fallback button for non-XR devices
    createFallbackVrButton();
  }
  showBackToDesktopButton();
}

function switchToDesktopView() {
  const desktopInterface = document.getElementById('desktopInterface');
  desktopInterface.style.display = 'block';
  hideBackToDesktopButton();
}

function showBackToDesktopButton() {
  let backBtn = document.getElementById('back-to-desktop-button');
  if (!backBtn) {
    backBtn = document.createElement('button');
    backBtn.id = 'back-to-desktop-button';
    backBtn.textContent = '← Back to Desktop View';
    backBtn.style.cssText = `
      position: fixed;
      bottom: 20px;
      left: 50%;
      transform: translateX(-50%);
      padding: 12px 24px;
      font-size: 16px;
      background-color: #666;
      color: white;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      z-index: 9998;
      box-shadow: 0 4px 8px rgba(0,0,0,0.3);
    `;
    backBtn.onclick = switchToDesktopView;
    document.body.appendChild(backBtn);
  }
  backBtn.style.display = 'block';
}

function hideBackToDesktopButton() {
  const backBtn = document.getElementById('back-to-desktop-button');
  if (backBtn) {
    backBtn.style.display = 'none';
  }
}

function createFallbackVrButton() {
  const startButton = document.createElement('button');
  startButton.id = 'start-tracking-button';
  startButton.textContent = 'Start Controller Tracking';
  startButton.style.cssText = `
    position: fixed;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    padding: 20px 40px;
    font-size: 20px;
    font-weight: bold;
    background-color: #4CAF50;
    color: white;
    border: none;
    border-radius: 8px;
    cursor: pointer;
    z-index: 9999;
    box-shadow: 0 4px 8px rgba(0,0,0,0.3);
  `;
  startButton.onclick = async () => {
    const sceneEl = document.querySelector('a-scene');
    if (sceneEl) {
      startButton.textContent = 'Connecting...';
      startButton.disabled = true;
      try {
        // Check if robot is already connected
        const statusResponse = await fetch('/api/status');
        const status = await statusResponse.json();
        if (!status.robotEngaged) {
          startButton.textContent = 'Connecting Arms...';
          const connectResponse = await fetch('/api/robot', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ action: 'connect' })
          });
          const connectResult = await connectResponse.json();
          if (!connectResult.success) {
            throw new Error(connectResult.error || 'Failed to connect robot arms');
          }
          await new Promise(resolve => setTimeout(resolve, 500));
        }
        startButton.textContent = 'Starting VR...';
        await sceneEl.enterVR(true);
      } catch (err) {
        alert(`Failed to start: ${err.message}`);
        startButton.textContent = 'Start Controller Tracking';
        startButton.disabled = false;
      }
    } else {
      alert('VR scene not available. Please reload the page.');
    }
  };
  document.body.appendChild(startButton);
} 