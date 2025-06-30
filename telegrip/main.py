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
    
    def should_redirect_to_tunnel(self):
        """Check if request should be redirected to tunnel URL."""
        # Only redirect main page requests, not API calls or assets
        if not (self.path == '/' or self.path == '/index.html'):
            return False
        
        # Check if tunnel is available
        tunnel_url = None
        if hasattr(self.server, 'api_handler') and self.server.api_handler:
            if hasattr(self.server.api_handler, 'tunnel_service'):
                tunnel_url = self.server.api_handler.tunnel_service.get_public_url()
        
        if not tunnel_url:
            return False
        
        # Check for VR headset User-Agent
        user_agent = self.headers.get('User-Agent', '').lower()
        vr_indicators = [
            'quest',           # Meta Quest browsers
            'oculus',          # Oculus browsers  
            'vr',              # Generic VR
            'headset',         # Generic headset
            'samsung internet', # Samsung VR browser
            'pico',            # Pico VR
            'htc',             # HTC Vive
            'varjo',           # Varjo headsets
            'openxr'           # OpenXR browsers
        ]
        
        is_vr_headset = any(indicator in user_agent for indicator in vr_indicators)
        
        if is_vr_headset:
            logger.info(f"🥽 VR headset detected ({user_agent[:50]}...), redirecting to tunnel: {tunnel_url}")
            
            # Send redirect response
            self.send_response(302)
            self.send_header('Location', tunnel_url + self.path)
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            return True
        
        return False
    
    def do_GET(self):
        """Handle GET requests."""
        # Check for VR headset and redirect to tunnel if available
        if self.should_redirect_to_tunnel():
            return
            
        if self.path == '/api/status':
            self.handle_status_request()
        elif self.path == '/api/config':
            self.handle_config_get_request()
        elif self.path == '/api/local-ip':
            self.handle_local_ip_request()
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
    
    def handle_local_ip_request(self):
        """Handle local IP address requests."""
        try:
            local_ip = get_local_ip()
            
            # Send JSON response
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            
            response = json.dumps({"local_ip": local_ip})
            self.wfile.write(response.encode('utf-8'))
            
        except Exception as e:
            logger.error(f"Error handling local IP request: {e}")
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
        """Serve a static file from the project directory."""
        from .utils import get_absolute_path
        try:
            # Convert relative path to absolute path in project directory
            abs_path = get_absolute_path(filename)
            
            with open(abs_path, 'rb') as f:
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
        server_type = "HTTP API"  # Default value
        try:
            # Create server - directly use APIHandler class
            self.httpd = http.server.HTTPServer((self.config.host_ip, self.config.https_port), APIHandler)
            
            # Set API handler reference for command queuing
            self.httpd.api_handler = self.system_ref
            
            # Setup SSL only in offline mode
            if self.config.offline_mode:
                context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                # Get absolute paths for SSL certificates
                cert_path, key_path = self.config.get_absolute_ssl_paths()
                context.load_cert_chain(cert_path, key_path)
                self.httpd.socket = context.wrap_socket(self.httpd.socket, server_side=True)
                server_type = "HTTPS"
            else:
                server_type = "HTTP API"
            
            # Start server in a separate thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()
            
            # Only log if INFO level or more verbose
            if getattr(logging, self.config.log_level.upper()) <= logging.INFO:
                logger.info(f"{server_type} server started on {self.config.host_ip}:{self.config.https_port}")
            
        except Exception as e:
            logger.error(f"Failed to start {server_type} server: {e}")
            raise
    
    async def stop(self):
        """Stop the HTTPS server."""
        if self.httpd:
            self.httpd.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=5)
            logger.info("HTTPS server stopped")


class HTTPRedirectServer:
    """Plain HTTP server that redirects users to tunnel (no certificate warnings)."""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        self.httpd = None
        self.server_thread = None
        self.system_ref = None  # Direct reference to the main system
    
    def set_system_ref(self, system_ref):
        """Set reference to the main teleoperation system."""
        self.system_ref = system_ref
    
    async def start(self):
        """Start the HTTP redirect server."""
        if self.config.offline_mode:
            return  # Only run when tunnel is enabled (default behavior)
            
        try:
            # Create redirect handler
            class RedirectHandler(http.server.BaseHTTPRequestHandler):
                def log_message(self, format, *args):
                    pass  # Disable logging
                
                def do_GET(self):
                    # Redirect all users to tunnel if available
                    if hasattr(self.server, 'system_ref') and self.server.system_ref:
                        tunnel_url = self.server.system_ref.tunnel_service.get_public_url()
                        if tunnel_url:
                            logger.debug(f"🌐 Redirecting to tunnel: {tunnel_url}")
                            self.send_response(302)
                            self.send_header('Location', tunnel_url + self.path)
                            self.send_header('Cache-Control', 'no-cache')
                            self.end_headers()
                            return
                    
                    # Tunnel not ready yet - show loading page
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html')
                    self.end_headers()
                    
                    html = f"""
                    <!DOCTYPE html>
                    <html>
                    <head>
                        <title>telegrip - Starting...</title>
                        <meta http-equiv="refresh" content="3">
                        <style>
                            body {{ font-family: Arial, sans-serif; text-align: center; padding: 50px; }}
                            .loading {{ color: #666; }}
                        </style>
                    </head>
                    <body>
                        <h1>🤖 telegrip</h1>
                        <p class="loading">⏳ Creating secure tunnel...</p>
                        <p><small>This page will automatically refresh</small></p>
                    </body>
                    </html>
                    """
                    self.wfile.write(html.encode('utf-8'))
            
            # Find available port (8080, 8081, etc.)
            http_port = 8080
            max_attempts = 10
            for attempt in range(max_attempts):
                try:
                    self.httpd = http.server.HTTPServer((self.config.host_ip, http_port), RedirectHandler)
                    self.httpd.system_ref = self.system_ref
                    break
                except OSError:
                    http_port += 1
                    if attempt == max_attempts - 1:
                        raise
            
            # Start server in a separate thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()
            
            logger.info(f"HTTP redirect server started on {self.config.host_ip}:{http_port}")
            
        except Exception as e:
            logger.error(f"Failed to start HTTP redirect server: {e}")
    
    async def stop(self):
        """Stop the HTTP redirect server."""
        if self.httpd:
            self.httpd.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=5)
            logger.info("HTTP redirect server stopped")


class TunnelService:
    """Service to create public tunnels using localhost.run or similar services."""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        self.tunnel_process = None
        self.tunnel_url = None
        self.is_running = False
        self.http_proxy_server = None
        self.http_proxy_thread = None
    
    def _start_http_proxy(self):
        """Start HTTP proxy server that forwards to HTTPS server."""
        import http.server
        import urllib.request
        import urllib.parse
        
        class ProxyHandler(http.server.BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                pass  # Disable logging
            
            def do_GET(self):
                self._proxy_request()
            
            def do_POST(self):
                self._proxy_request()
            
            def do_OPTIONS(self):
                self._proxy_request()
            
            def _proxy_request(self):
                try:
                    # Build target URL (HTTP in default mode, HTTPS in offline mode)
                    if self.server.offline_mode:
                        target_url = f"https://localhost:{self.server.https_port}{self.path}"
                    else:
                        target_url = f"http://localhost:{self.server.https_port}{self.path}"
                    
                    # Create request
                    request = urllib.request.Request(target_url)
                    
                    # Copy headers (except Host)
                    for header, value in self.headers.items():
                        if header.lower() not in ['host', 'connection']:
                            request.add_header(header, value)
                    
                    # Handle POST data
                    if self.command == 'POST':
                        content_length = int(self.headers.get('Content-Length', 0))
                        post_data = self.rfile.read(content_length)
                        request.data = post_data
                    
                    # Make request with SSL context only for HTTPS
                    if self.server.offline_mode:
                        # HTTPS with self-signed cert - disable verification
                        import ssl
                        context = ssl.create_default_context()
                        context.check_hostname = False
                        context.verify_mode = ssl.CERT_NONE
                        response = urllib.request.urlopen(request, context=context)
                    else:
                        # HTTP - no SSL context needed
                        response = urllib.request.urlopen(request)
                    
                    # Forward response
                    self.send_response(response.getcode())
                    
                    # Copy response headers
                    for header, value in response.headers.items():
                        if header.lower() not in ['connection', 'transfer-encoding']:
                            self.send_header(header, value)
                    
                    self.end_headers()
                    
                    # Copy response body
                    self.wfile.write(response.read())
                    
                except Exception as e:
                    logger.debug(f"Proxy error: {e}")
                    self.send_error(502, "Bad Gateway")
        
        # Find available port for HTTP proxy
        proxy_port = 8080
        max_attempts = 10
        for attempt in range(max_attempts):
            try:
                self.http_proxy_server = http.server.HTTPServer(('localhost', proxy_port), ProxyHandler)
                self.http_proxy_server.https_port = self.config.https_port  # Pass HTTPS port to handler
                self.http_proxy_server.offline_mode = self.config.offline_mode  # Pass offline mode to handler
                break
            except OSError:
                proxy_port += 1
                if attempt == max_attempts - 1:
                    raise
        
        # Start proxy server in thread
        self.http_proxy_thread = threading.Thread(
            target=self.http_proxy_server.serve_forever, 
            daemon=True
        )
        self.http_proxy_thread.start()
        
        logger.debug(f"HTTP proxy server started on port {proxy_port}")
        return proxy_port

    async def start(self):
        """Start the tunnel service."""
        if self.config.offline_mode:
            return
        
        try:
            logger.info("🌐 Creating public tunnel...")
            
            # Start HTTP proxy server first
            proxy_port = self._start_http_proxy()
            
            # Command to create SSH tunnel to localhost.run (forward to HTTP proxy)
            cmd = [
                "ssh", 
                "-R", f"80:localhost:{proxy_port}",
                "-o", "StrictHostKeyChecking=no",
                "-o", "UserKnownHostsFile=/dev/null",
                "-o", "LogLevel=ERROR",
                "nokey@localhost.run"
            ]
            
            # Start the SSH process
            self.tunnel_process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE
            )
            
            # Wait for tunnel URL from stdout/stderr (with timeout)
            try:
                # Read output to get the tunnel URL
                url_found = False
                timeout_time = asyncio.get_event_loop().time() + 15.0
                
                async def read_stream(stream, stream_name):
                    nonlocal url_found
                    while not url_found and asyncio.get_event_loop().time() < timeout_time:
                        try:
                            line = await asyncio.wait_for(stream.readline(), timeout=1.0)
                            if line:
                                output = line.decode('utf-8').strip()
                                if output:  # Only log non-empty lines
                                    logger.debug(f"Tunnel {stream_name}: {output}")
                                    
                                    # Look for the tunnel URL in the output
                                    import re
                                    # Match various localhost.run URL formats
                                    url_patterns = [
                                        r'https://[a-zA-Z0-9\-]+\.localhost\.run',
                                        r'http://[a-zA-Z0-9\-]+\.localhost\.run',
                                        r'https://[a-zA-Z0-9\-]+\.lhr\.life',
                                        r'http://[a-zA-Z0-9\-]+\.lhr\.life',
                                        r'Connect to (https?://[a-zA-Z0-9\-]+\.(?:localhost\.run|lhr\.life))',
                                        r'tunneled.*?(https?://[a-zA-Z0-9\-]+\.(?:localhost\.run|lhr\.life))'
                                    ]
                                    
                                    for pattern in url_patterns:
                                        match = re.search(pattern, output, re.IGNORECASE)
                                        if match:
                                            if match.groups():
                                                self.tunnel_url = match.group(1)
                                            else:
                                                self.tunnel_url = match.group(0)
                                            
                                            # Ensure HTTPS
                                            if self.tunnel_url.startswith('http://'):
                                                self.tunnel_url = self.tunnel_url.replace('http://', 'https://', 1)
                                            
                                            self.is_running = True
                                            url_found = True
                                            logger.info(f"🌐 Tunnel created successfully: {self.tunnel_url}")
                                            return
                            else:
                                # No more output, wait briefly
                                await asyncio.sleep(0.1)
                        except asyncio.TimeoutError:
                            # No output yet, continue waiting
                            continue
                        except Exception as e:
                            logger.debug(f"Error reading {stream_name}: {e}")
                            break
                
                # Read from both stdout and stderr concurrently
                await asyncio.gather(
                    read_stream(self.tunnel_process.stdout, "stdout"),
                    read_stream(self.tunnel_process.stderr, "stderr"),
                    return_exceptions=True
                )
                
                if not url_found:
                    if self.tunnel_process.returncode is None:
                        logger.warning("Could not get tunnel URL within timeout, but process is still running")
                        # Process is running - assume tunnel might be working, we just can't parse the URL
                        # Try a common localhost.run format as fallback
                        logger.info("🌐 Tunnel process running - check your SSH client for the actual URL")
                    else:
                        logger.warning(f"Tunnel process exited with code: {self.tunnel_process.returncode}")
                    
            except Exception as e:
                logger.warning(f"Error reading tunnel URL: {e}")
                if self.tunnel_process:
                    self.tunnel_process.terminate()
                    
        except FileNotFoundError:
            logger.error("SSH not found. Please install SSH client to use --online option.")
            logger.error("On Ubuntu/Debian: sudo apt-get install openssh-client")
            logger.error("On Windows: Enable OpenSSH client in Windows Features")
        except Exception as e:
            logger.error(f"Failed to create tunnel: {e}")
    
    async def stop(self):
        """Stop the tunnel service."""
        self.is_running = False
        
        # Stop HTTP proxy server
        if self.http_proxy_server:
            self.http_proxy_server.shutdown()
            if self.http_proxy_thread:
                self.http_proxy_thread.join(timeout=2)
        
        # Stop SSH tunnel
        if self.tunnel_process and self.tunnel_process.returncode is None:
            self.tunnel_process.terminate()
            try:
                await asyncio.wait_for(self.tunnel_process.wait(), timeout=5.0)
            except asyncio.TimeoutError:
                self.tunnel_process.kill()
            logger.info("🌐 Tunnel stopped")
    
    def get_public_url(self) -> Optional[str]:
        """Get the public tunnel URL."""
        return self.tunnel_url if self.is_running else None


class TelegripSystem:
    """Main teleoperation system that coordinates all components."""
    
    def __init__(self, config: TelegripConfig):
        self.config = config
        
        # Command queues
        self.command_queue = asyncio.Queue()
        self.control_commands_queue = queue.Queue(maxsize=10)  # Thread-safe queue
        
        # Components
        self.https_server = HTTPSServer(config)
        self.http_redirect_server = HTTPRedirectServer(config)
        self.tunnel_service = TunnelService(config)
        self.vr_server = VRWebSocketServer(self.command_queue, config)
        self.keyboard_listener = KeyboardListener(self.command_queue, config)
        self.control_loop = ControlLoop(self.command_queue, config, self.control_commands_queue)
        
        # Set system reference for API calls
        self.https_server.set_system_ref(self)
        self.http_redirect_server.set_system_ref(self)
        
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
            # Don't stop tunnel service, HTTP redirect server, or HTTPS server - keep them running for the UI
            
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
            # Note: Don't recreate tunnel service - keep the existing one running
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
            
            # Auto-connect to robot if requested (preserve autoconnect behavior after restart)
            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 Auto-connecting to robot motors after restart...")
                await asyncio.sleep(0.5)  # Brief delay to let components settle
                self.add_control_command("robot_connect")
            
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
            
            # Start HTTP redirect server (if tunnel enabled)
            await self.http_redirect_server.start()
            
            # Start tunnel service (if enabled)
            await self.tunnel_service.start()
            
            # Show public URL if tunnel was created successfully
            if self.tunnel_service.get_public_url():
                public_url = self.tunnel_service.get_public_url()
                current_log_level = getattr(logging, self.config.log_level.upper())
                if current_log_level <= logging.INFO:
                    logger.info(f"Public tunnel created: {public_url}")
                else:
                    # Show the public URL prominently in quiet mode
                    print(f"🌐 Public URL (use this on VR headset - no warnings!):")
                    print(f"   {public_url}")
                    print()
            
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
            
            # Auto-connect to robot if requested
            if self.config.autoconnect and self.config.enable_robot:
                logger.info("🔌 Auto-connecting to robot motors...")
                await asyncio.sleep(0.5)  # Brief delay to let components settle
                self.add_control_command("robot_connect")
            
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
        await self.tunnel_service.stop()
        await self.http_redirect_server.stop()
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
    parser.add_argument("--autoconnect", action="store_true", help="Automatically connect to robot motors on startup")
    parser.add_argument("--offline", action="store_true", help="Use self-signed certificates (offline mode - shows browser warnings)")
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
    config.autoconnect = args.autoconnect
    config.offline_mode = args.offline
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
        logger.info(f"  Auto-connect: {'enabled' if config.autoconnect else 'disabled'}")
        logger.info(f"  HTTPS Port: {config.https_port}")
        logger.info(f"  WebSocket Port: {config.websocket_port}")
        logger.info(f"  Robot Ports: {config.follower_ports}")
        logger.info(f"  Offline mode: {'enabled' if config.offline_mode else 'disabled'}")
    else:
        # Show clean startup message with HTTPS URL
        host_display = get_local_ip() if config.host_ip == "0.0.0.0" else config.host_ip
        print(f"🤖 telegrip starting...")
        if config.offline_mode:
            print(f"📱 Offline mode - Open the UI in your browser on:")
            print(f"   https://{host_display}:{config.https_port}")
            print(f"⚠️  You may need to accept certificate warnings")
        else:
            print(f"📱 Open the UI in your browser on:")
            print(f"   http://{host_display}:8080")
            print(f"🌐 Creating secure tunnel (no certificate warnings)...")
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
