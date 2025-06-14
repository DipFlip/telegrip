"""
Main entry point for the unified teleoperation system.
Coordinates HTTPS server, WebSocket server, robot interface, and input providers.
"""

import asyncio
import argparse
import logging
import signal
import sys
import os
import http.server
import ssl
import socket
import json
import urllib.parse
import time
import contextlib
from typing import Optional
import queue  # Add regular queue for thread-safe communication
import threading
from pathlib import Path
import weakref


def get_local_ip():
    """Get the local IP address of this machine."""
    try:
        # Connect to a remote address to determine the local IP
        # This doesn't actually send any data
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        try:
            # Fallback: get hostname IP
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            # Final fallback
            return "localhost"


@contextlib.contextmanager
def suppress_stdout_stderr():
    """Context manager to suppress stdout and stderr output at the file descriptor level."""
    # Save original file descriptors
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()
    
    # Save original file descriptors
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)
    
    try:
        # Open devnull
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        
        # Redirect stdout and stderr to devnull
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)
        
        yield
        
    finally:
        # Restore original file descriptors
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)
        
        # Close saved file descriptors
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


# Import telegrip modules after function definition
from .config import TelegripConfig, get_config_data, update_config_data
from .control_loop import ControlLoop
from .inputs.vr_ws_server import VRWebSocketServer
from .inputs.keyboard_listener import KeyboardListener
from .inputs.base import ControlGoal

# Logger will be configured in main() based on command line arguments
logger = logging.getLogger(__name__)


class APIHandler(http.server.BaseHTTPRequestHandler):
    """HTTP request handler for the teleoperation API."""
    
    def __init__(self, *args, **kwargs):
        # Set CORS headers for all requests
        super().__init__(*args, **kwargs)
    
    def end_headers(self):
        """Add CORS headers to all responses."""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        try:
            super().end_headers()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ssl.SSLError):
            # Client disconnected or SSL error - ignore silently
            pass
    
    def do_OPTIONS(self):
        """Handle preflight CORS requests."""
        self.send_response(200)
        self.end_headers()
    
    def log_message(self, format, *args):
        """Override to reduce HTTP request logging noise."""
        pass  # Disable default HTTP logging
    
    def do_GET(self):
        """Handle GET requests."""
        if self.path == '/api/status':
            self.handle_status_request()
        elif self.path == '/api/config':
            self.handle_config_get_request()
        elif self.path == '/' or self.path == '/index.html':
            # Serve main page from web-ui directory
            self.serve_file('web-ui/index.html', 'text/html')
        elif self.path.endswith('.css'):
            # Serve CSS files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'text/css')
        elif self.path.endswith('.js'):
            # Serve JS files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'application/javascript')
        elif self.path.endswith('.ico'):
            self.serve_file(self.path[1:], 'image/x-icon')
        elif self.path.endswith(('.jpg', '.jpeg')):
            # Serve image files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'image/jpeg')
        elif self.path.endswith('.png'):
            # Serve image files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'image/png')
        elif self.path.endswith('.gif'):
            # Serve image files from web-ui directory
            self.serve_file(f'web-ui{self.path}', 'image/gif')
        else:
            self.send_error(404, "Not found")
    
    def do_POST(self):
        """Handle POST requests."""
        if self.path == '/api/keyboard':
            self.handle_keyboard_request()
        elif self.path == '/api/robot':
            self.handle_robot_request()
        elif self.path == '/api/keypress':
            self.handle_keypress_request()
        elif self.path == '/api/config':
            self.handle_config_post_request()
        elif self.path == '/api/restart':
            self.handle_restart_request()
        else:
            self.send_error(404, "Not found")
    
    def handle_status_request(self):
        """Handle status requests."""
        try:
            # Get system reference
            if hasattr(self.server, 'api_handler') and self.server.api_handler:
                system = self.server.api_handler
                
                # Get status from control loop
                control_status = system.control_loop.status if system.control_loop else {}
                
                # Get keyboard status
                keyboard_enabled = False
                if system.keyboard_listener and hasattr(system.keyboard_listener, 'is_enabled'):
                    keyboard_enabled = system.keyboard_listener.is_enabled
                
                # Get robot engagement status
                robot_engaged = False
                if system.control_loop and system.control_loop.robot_interface:
                    robot_engaged = system.control_loop.robot_interface.is_engaged
                
                # Get VR connection status
                vr_connected = False
                if system.vr_server and system.vr_server.is_running:
                    vr_connected = len(system.vr_server.clients) > 0
                
                status = {
                    **control_status,
                    "keyboardEnabled": keyboard_enabled,
                    "robotEngaged": robot_engaged,
                    "vrConnected": vr_connected
                }
                
                # Send JSON response
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                
                response = json.dumps(status)
                self.wfile.write(response.encode('utf-8'))
            else:
                self.send_error(500, "System not available")
                
        except Exception as e:
            logger.error(f"Error handling status request: {e}")
            self.send_error(500, str(e))
    
    def handle_keyboard_request(self):
        """Handle keyboard control requests."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "No request body")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            action = data.get('action')
            
            if action in ['enable', 'disable']:
                # Add keyboard command to queue for processing by main thread
                if hasattr(self.server, 'api_handler') and self.server.api_handler:
                    command_name = f"{action}_keyboard"
                    logger.info(f"🎮 Adding command to queue: {command_name}")
                    self.server.api_handler.add_control_command(command_name)
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({"success": True, "action": action}).encode('utf-8'))
                else:
                    self.send_error(500, "System not available")
            else:
                self.send_error(400, f"Invalid action: {action}")
                
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error handling keyboard request: {e}")
            self.send_error(500, str(e))
    
    def handle_robot_request(self):
        """Handle robot control requests."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "No request body")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            action = data.get('action')
            logger.info(f"🔌 Received robot action: {action}")
            
            if action in ['connect', 'disconnect']:
                # Add robot command to queue for processing by main thread
                if hasattr(self.server, 'api_handler') and self.server.api_handler:
                    command_name = f"robot_{action}"
                    logger.info(f"🔌 Adding command to queue: {command_name}")
                    self.server.api_handler.add_control_command(command_name)
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({"success": True, "action": action}).encode('utf-8'))
                else:
                    logger.error("🔌 Server api_handler not available")
                    self.send_error(500, "System not available")
            else:
                self.send_error(400, f"Invalid action: {action}")
                
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error handling robot request: {e}")
            self.send_error(500, str(e))
    
    def handle_keypress_request(self):
        """Handle keypress control requests."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "No request body")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            key = data.get('key')
            action = data.get('action')
            
            if key and action in ['press', 'release']:
                # Add keypress command to queue for processing by main thread
                if hasattr(self.server, 'api_handler') and self.server.api_handler:
                    command = {
                        "action": "web_keypress",
                        "key": key,
                        "event": action
                    }
                    logger.info(f"🎮 Adding keypress command to queue: {key}_{action}")
                    self.server.api_handler.add_keypress_command(command)
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')  
                    self.end_headers()
                    self.wfile.write(json.dumps({"success": True, "key": key, "action": action}).encode('utf-8'))
                else:
                    logger.error("🎮 Server api_handler not available")
                    self.send_error(500, "System not available")
            else:
                self.send_error(400, f"Invalid key or action: {key}, {action}")
                
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error handling keypress request: {e}")
            self.send_error(500, str(e))
    
    def handle_config_get_request(self):
        """Handle configuration read requests."""
        try:
            config_data = get_config_data()
            
            # Send JSON response
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            
            response = json.dumps(config_data)
            self.wfile.write(response.encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error handling config get request: {e}")
            self.send_error(500, str(e))
    
    def handle_config_post_request(self):
        """Handle configuration update requests."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "No request body")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            # Update configuration
            success = update_config_data(data)
            
            if success:
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"success": True, "message": "Configuration updated successfully"}).encode('utf-8'))
                logger.info("Configuration updated successfully")
            else:
                self.send_error(500, "Failed to save configuration")
                
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error handling config post request: {e}")
            self.send_error(500, str(e))
    
    def handle_restart_request(self):
        """Handle restart requests."""
        try:
            if hasattr(self.server, 'api_handler') and self.server.api_handler:
                logger.info("Restarting teleoperation system...")
                self.server.api_handler.restart()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"success": True, "message": "Teleoperation system restarted"}).encode('utf-8'))
            else:
                self.send_error(500, "System not available")
                
        except Exception as e:
            logger.error(f"Error handling restart request: {e}")
            self.send_error(500, str(e))
    
    def serve_file(self, filename, content_type):
        """Serve a static file from the current directory."""
        try:
            with open(filename, 'rb') as f:
                file_content = f.read()
            
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', len(file_content))
            self.end_headers()
            self.wfile.write(file_content)
            
        except FileNotFoundError:
            self.send_error(404, f"File {filename} not found")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            # Client disconnected - log quietly and continue
            logger.debug(f"Client disconnected while serving {filename}")
        except Exception as e:
            logger.error(f"Error serving file {filename}: {e}")
            try:
                self.send_error(500, "Internal server error")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                # Client already disconnected, ignore
                pass


class HTTPSServer:
    """HTTPS server for the teleoperation API."""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        self.httpd = None
        self.server_thread = None
        self.system_ref = None  # Direct reference to the main system
    
    def set_system_ref(self, system_ref):
        """Set reference to the main teleoperation system."""
        self.system_ref = system_ref
    
    async def start(self):
        """Start the HTTPS server."""
        try:
            # Create server - directly use APIHandler class
            self.httpd = http.server.HTTPServer((self.config.host_ip, self.config.https_port), APIHandler)
            
            # Set API handler reference for command queuing
            self.httpd.api_handler = self.system_ref
            
            # Setup SSL
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain('cert.pem', 'key.pem')
            self.httpd.socket = context.wrap_socket(self.httpd.socket, server_side=True)
            
            # Start server in a separate thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()
            
            # Only log if INFO level or more verbose
            if getattr(logging, self.config.log_level.upper()) <= logging.INFO:
                logger.info(f"HTTPS server started on {self.config.host_ip}:{self.config.https_port}")
            
        except Exception as e:
            logger.error(f"Failed to start HTTPS server: {e}")
            raise
    
    async def stop(self):
        """Stop the HTTPS server."""
        if self.httpd:
            self.httpd.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=5)
            logger.info("HTTPS server stopped")


class TelegripSystem:
    """Main teleoperation system that coordinates all components."""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        
        # Command queues
        self.command_queue = asyncio.Queue()
        self.control_commands_queue = queue.Queue(maxsize=10)  # Thread-safe queue
        
        # Components
        self.https_server = HTTPSServer(config)
        self.vr_server = VRWebSocketServer(self.command_queue, config)
        self.keyboard_listener = KeyboardListener(self.command_queue, config)
        self.control_loop = ControlLoop(self.command_queue, config, self.control_commands_queue)
        
        # Set system reference for API calls
        self.https_server.set_system_ref(self)
        
        # Set up cross-references
        self.control_loop.keyboard_listener = self.keyboard_listener
        
        # Tasks
        self.tasks = []
        self.is_running = False
        self.main_loop = None  # Will be set when the system starts
    
    def add_control_command(self, action: str):
        """Add a control command to the queue for processing."""
        try:
            command = {"action": action}
            logger.info(f"🔌 Queueing control command: {command}")
            self.control_commands_queue.put_nowait(command)
            logger.info(f"🔌 Command queued successfully")
        except queue.Full:
            logger.warning(f"Control commands queue is full, dropping command: {action}")
        except Exception as e:
            logger.error(f"🔌 Error queuing command: {e}")
    
    def add_keypress_command(self, command: dict):
        """Add a keypress command to the queue for processing."""
        try:
            logger.info(f"🎮 Queueing keypress command: {command}")
            self.control_commands_queue.put_nowait(command)
            logger.info(f"🎮 Keypress command queued successfully")
        except queue.Full:
            logger.warning(f"Control commands queue is full, dropping keypress command: {command}")
        except Exception as e:
            logger.error(f"🎮 Error queuing keypress command: {e}")
    
    async def process_control_commands(self):
        """Process control commands from the thread-safe queue."""
        try:
            # Get all available commands from the thread-safe queue
            commands_to_process = []
            while True:
                try:
                    command = self.control_commands_queue.get_nowait()
                    commands_to_process.append(command)
                except queue.Empty:
                    break
            
            # Process each command
            for command in commands_to_process:
                if self.control_loop:
                    await self.control_loop._handle_command(command)
                    
        except Exception as e:
            logger.error(f"Error processing control commands: {e}")
    
    def restart(self):
        """Restart the teleoperation system."""
        def do_restart():
            try:
                logger.info("Initiating system restart...")
                # Use the stored main event loop reference to schedule the soft restart
                if self.main_loop and not self.main_loop.is_closed():
                    future = asyncio.run_coroutine_threadsafe(self._soft_restart_sequence(), self.main_loop)
                    # Wait for restart to complete
                    future.result(timeout=30.0)
                else:
                    logger.error("Main event loop not available for restart")
            except Exception as e:
                logger.error(f"Error during restart: {e}")
        
        # Run restart in a separate thread to avoid blocking the HTTP response
        restart_thread = threading.Thread(target=do_restart, daemon=True)
        restart_thread.start()
    
    async def _soft_restart_sequence(self):
        """Perform a soft restart by reinitializing components without exiting the process."""
        try:
            logger.info("Starting soft restart sequence...")
            
            # Wait a moment to let the HTTP response be sent
            await asyncio.sleep(1)
            
            # Cancel all tasks
            for task in self.tasks:
                task.cancel()
            
            # Wait for tasks to complete with timeout
            if self.tasks:
                try:
                    await asyncio.wait_for(
                        asyncio.gather(*self.tasks, return_exceptions=True), 
                        timeout=5.0
                    )
                except asyncio.TimeoutError:
                    logger.warning("Some tasks did not complete within timeout")
            
            # Stop components in reverse order
            await self.control_loop.stop()
            await self.keyboard_listener.stop()
            await self.vr_server.stop()
            # Don't stop HTTPS server - keep it running for the UI
            
            # Wait a moment for cleanup
            await asyncio.sleep(1)
            
            # Reload configuration from file but preserve command-line overrides
            from .config import get_config_data
            file_config = get_config_data()
            logger.info("Configuration reloaded from file")
            
            # Keep the existing configuration object to preserve command-line arguments
            # Just update specific values that might have changed in the config file
            
            # Recreate components with existing configuration
            self.command_queue = asyncio.Queue()
            self.control_commands_queue = queue.Queue(maxsize=10)
            
            # Create new components
            self.vr_server = VRWebSocketServer(self.command_queue, self.config)
            self.keyboard_listener = KeyboardListener(self.command_queue, self.config)
            self.control_loop = ControlLoop(self.command_queue, self.config, self.control_commands_queue)
            
            # Set up cross-references
            self.control_loop.keyboard_listener = self.keyboard_listener
            
            # Clear old tasks
            self.tasks = []
            
            # Start VR WebSocket server
            await self.vr_server.start()
            
            # Start keyboard listener
            await self.keyboard_listener.start()
            
            # Start control loop
            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)
            
            # Start control command processor
            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)
            
            logger.info("System restart completed successfully")
            
        except Exception as e:
            logger.error(f"Error during soft restart sequence: {e}")
            raise
    
    async def start(self):
        """Start all system components."""
        try:
            self.is_running = True
            
            # Store reference to the main event loop for restart functionality
            self.main_loop = asyncio.get_event_loop()
            
            # Start HTTPS server
            await self.https_server.start()
            
            # Start VR WebSocket server
            await self.vr_server.start()
            
            # Start keyboard listener
            await self.keyboard_listener.start()
            
            # Start control loop
            control_task = asyncio.create_task(self.control_loop.start())
            self.tasks.append(control_task)
            
            # Start control command processor
            command_processor_task = asyncio.create_task(self._run_command_processor())
            self.tasks.append(command_processor_task)
            
            logger.info("All system components started successfully")
            
            # Main loop that handles restarts
            while self.is_running:
                try:
                    # Wait for tasks to complete
                    await asyncio.gather(*self.tasks)
                    # If we get here, all tasks completed normally (shouldn't happen in normal operation)
                    break
                except asyncio.CancelledError:
                    # Tasks were cancelled - check if it's due to restart
                    if self.is_running:
                        # System is restarting, wait for restart to complete
                        await asyncio.sleep(1)
                        # Continue the loop to wait for new tasks
                        continue
                    else:
                        # Normal shutdown
                        break
                except Exception as e:
                    logger.error(f"Error in main task loop: {e}")
                    break
            
        except Exception as e:
            logger.error(f"Error starting teleoperation system: {e}")
            await self.stop()
            raise
    
    async def _run_command_processor(self):
        """Run the control command processor loop."""
        while self.is_running:
            await self.process_control_commands()
            await asyncio.sleep(0.05)  # Check for commands every 50ms
    
    async def stop(self):
        """Stop all system components."""
        logger.info("Shutting down teleoperation system...")
        self.is_running = False
        
        # Cancel all tasks
        for task in self.tasks:
            task.cancel()
        
        # Wait for tasks to complete with timeout
        if self.tasks:
            try:
                await asyncio.wait_for(
                    asyncio.gather(*self.tasks, return_exceptions=True), 
                    timeout=5.0
                )
            except asyncio.TimeoutError:
                logger.warning("Some tasks did not complete within timeout")
        
        # Stop components in reverse order
        await self.control_loop.stop()
        await self.keyboard_listener.stop()
        await self.vr_server.stop()
        await self.https_server.stop()
        
        logger.info("Teleoperation system shutdown complete")


def signal_handler(signum, frame):
    """Handle shutdown signals."""
    logger.info(f"Received signal {signum}")
    raise KeyboardInterrupt()


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Unified SO100 Robot Teleoperation System")
    
    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument("--no-sim", action="store_true", help="Disable PyBullet simulation and inverse kinematics")
    parser.add_argument("--no-viz", action="store_true", help="Disable PyBullet visualization (headless mode)")
    parser.add_argument("--no-vr", action="store_true", help="Disable VR WebSocket server")
    parser.add_argument("--no-keyboard", action="store_true", help="Disable keyboard input")
    parser.add_argument("--no-https", action="store_true", help="Disable HTTPS server")
    parser.add_argument("--log-level", default="warning", 
                       choices=["debug", "info", "warning", "error", "critical"],
                       help="Set logging level (default: warning)")
    
    # Network settings
    parser.add_argument("--https-port", type=int, default=8443, help="HTTPS server port")
    parser.add_argument("--ws-port", type=int, default=8442, help="WebSocket server port")
    parser.add_argument("--host", default="0.0.0.0", help="Host IP address")
    
    # Paths
    parser.add_argument("--urdf", default="URDF/SO100/so100.urdf", help="Path to robot URDF file")
    parser.add_argument("--webapp", default="webapp", help="Path to webapp directory")
    parser.add_argument("--cert", default="cert.pem", help="Path to SSL certificate")
    parser.add_argument("--key", default="key.pem", help="Path to SSL private key")
    
    # Robot settings
    parser.add_argument("--config", default="config.yaml", help="Path to config file")
    parser.add_argument("--left-port", help="Left arm serial port (overrides config file)")
    parser.add_argument("--right-port", help="Right arm serial port (overrides config file)")
    
    return parser.parse_args()


def create_config_from_args(args) -> TelegripConfig:
    """Create configuration object from command line arguments."""
    # First load the config file
    config_data = get_config_data()
    config = TelegripConfig()
    
    # Apply command line overrides
    config.enable_robot = not args.no_robot
    config.enable_pybullet = not args.no_sim
    config.enable_pybullet_gui = config.enable_pybullet and not args.no_viz
    config.enable_vr = not args.no_vr
    config.enable_keyboard = not args.no_keyboard
    config.log_level = args.log_level
    
    config.https_port = args.https_port
    config.websocket_port = args.ws_port
    config.host_ip = args.host
    
    config.urdf_path = args.urdf
    config.webapp_dir = args.webapp
    config.certfile = args.cert
    config.keyfile = args.key
    
    # Handle port configuration - use command line args if provided, otherwise use config file values
    if args.left_port or args.right_port:
        config.follower_ports = {
            "left": args.left_port if args.left_port else config_data["robot"]["left_arm"]["port"],
            "right": args.right_port if args.right_port else config_data["robot"]["right_arm"]["port"]
        }
    
    return config


async def main():
    """Main entry point."""
    # Parse arguments first to check for log level
    args = parse_arguments()
    
    # Setup logging based on log level
    log_level = getattr(logging, args.log_level.upper())
    
    # Suppress PyBullet's native output when not in verbose mode
    if log_level > logging.INFO:
        os.environ['PYBULLET_SUPPRESS_CONSOLE_OUTPUT'] = '1'
        os.environ['PYBULLET_SUPPRESS_WARNINGS'] = '1'
    
    if log_level <= logging.INFO:
        # Verbose mode - show detailed logging with timestamps
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    else:
        # Quiet mode - only show warnings and errors with simple format
        logging.basicConfig(
            level=log_level,
            format='%(message)s'
        )
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    config = create_config_from_args(args)
    
    # Log configuration (only if INFO level or more verbose)
    if log_level <= logging.INFO:
        logger.info("Starting with configuration:")
        logger.info(f"  Robot: {'enabled' if config.enable_robot else 'disabled'}")
        logger.info(f"  PyBullet: {'enabled' if config.enable_pybullet else 'disabled'}")
        logger.info(f"  Headless mode: {'enabled' if not config.enable_pybullet_gui and config.enable_pybullet else 'disabled'}")
        logger.info(f"  VR: {'enabled' if config.enable_vr else 'disabled'}")
        logger.info(f"  Keyboard: {'enabled' if config.enable_keyboard else 'disabled'}")
        logger.info(f"  HTTPS Port: {config.https_port}")
        logger.info(f"  WebSocket Port: {config.websocket_port}")
        logger.info(f"  Robot Ports: {config.follower_ports}")
    else:
        # Show clean startup message with HTTPS URL
        host_display = get_local_ip() if config.host_ip == "0.0.0.0" else config.host_ip
        print(f"🤖 telegrip starting...")
        print(f"📱 Open your VR headset browser and navigate to:")
        print(f"   https://{host_display}:{config.https_port}")
        print(f"💡 Use --log-level info to see detailed output")
        print()
    
    # Create and start teleoperation system
    system = TelegripSystem(config)
    
    try:
        await system.start()
    except KeyboardInterrupt:
        if log_level <= logging.INFO:
            logger.info("Received interrupt signal")
        else:
            print("\n🛑 Shutting down...")
    except asyncio.CancelledError:
        # Handle cancelled error (often from restart scenarios)
        if log_level <= logging.INFO:
            logger.info("System tasks cancelled")
    except Exception as e:
        if log_level <= logging.INFO:
            logger.error(f"System error: {e}")
        else:
            print(f"❌ Error: {e}")
    finally:
        try:
            await system.stop()
        except asyncio.CancelledError:
            # Ignore cancelled errors during shutdown
            pass
        if log_level > logging.INFO:
            print("✅ Shutdown complete.")


def main_cli():
    """Console script entry point for pip-installed package."""
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown complete.")
    except asyncio.CancelledError:
        # Handle cancelled error from restart scenarios  
        pass
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main_cli() 
