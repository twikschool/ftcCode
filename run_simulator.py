import http.server
import socketserver
import webbrowser
import os
from pathlib import Path

PORT = 8080
SIMULATOR_DIR = Path(__file__).parent / "simulator"

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(SIMULATOR_DIR), **kwargs)

def start_simulator():
    print("================================================================")
    print("🐝 Étude Engineers - FTC BioBuzz Robot Coding Simulator")
    print(f"📡 Starting local server at http://localhost:{PORT}")
    print("================================================================")
    
    # Open browser automatically
    webbrowser.open(f"http://localhost:{PORT}")
    
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down simulator server.")

if __name__ == "__main__":
    start_simulator()

