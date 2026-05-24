import http.server
import socketserver
import subprocess
import os

PORT = 8081

class MyHandler(http.server.SimpleHTTPRequestHandler):
    def do_POST(self):
        # On intercepte la route du bouton
        if self.path == '/api/launch-exe':
            try:
                root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
                # Lance ton exécutable en arrière-plan sans bloquer le serveur
                # Remplace 'mon_programme.exe' par le nom de ton script ou exécutable
                subprocess.Popen(['./test_generation'], cwd=root_dir) 
                
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"OK")
            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(str(e).encode())

        elif self.path == '/api/launch-exe-test':
            try:
                root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
                # Lance ton exécutable en arrière-plan sans bloquer le serveur
                # Remplace 'mon_programme.exe' par le nom de ton script ou exécutable
                subprocess.Popen(['./droneTest'], cwd=root_dir) 
                
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

# Lance le serveur (il gère le HTML/JS automatiquement + le bouton POST)
socketserver.TCPServer.allow_reuse_address = True
with socketserver.TCPServer(("", PORT), MyHandler) as httpd:
    print(f"Serveur actif sur http://localhost:{PORT}")
    httpd.serve_forever()