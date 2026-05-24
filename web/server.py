import http.server
import socketserver
import os

PORT = 8081

class MyHandler(http.server.SimpleHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/api/launch-main':
            try:
                root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
                flag_path = os.path.join(root_dir, 'reset.flag')
                
                open(flag_path, 'a').close()
                
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"OK")
            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(str(e).encode())
        else:
            self.send_response(404)
            self.end_headers()

socketserver.TCPServer.allow_reuse_address = True
with socketserver.TCPServer(("", PORT), MyHandler) as httpd:
    print(f"Serveur GCS actif sur http://localhost:{PORT}")
    httpd.serve_forever()
