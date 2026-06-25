#!/usr/bin/env python3
"""Serve the AutoNav web visualizer over HTTPS for local development.

Requires a locally-trusted certificate from mkcert (https://github.com/FiloSottile/mkcert):

    mkcert -install          # once, installs the local CA into the OS trust store
    mkcert localhost         # generates localhost.pem + localhost-key.pem here

Then run this script from the web/ folder:

    python serve_https.py

and open https://localhost:8443/maze.html
"""
import http.server
import ssl
import sys
import os

CERT = "localhost.pem"
KEY = "localhost-key.pem"
PORT = 8443

if not (os.path.exists(CERT) and os.path.exists(KEY)):
    sys.exit(
        f"Missing {CERT} / {KEY}.\n"
        "Generate them with:  mkcert -install && mkcert localhost\n"
        "and run this script from the folder that contains them."
    )

ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ctx.load_cert_chain(CERT, KEY)

srv = http.server.HTTPServer(("127.0.0.1", PORT), http.server.SimpleHTTPRequestHandler)
srv.socket = ctx.wrap_socket(srv.socket, server_side=True)

print(f"Serving https://localhost:{PORT}/maze.html  (Ctrl+C to stop)")
try:
    srv.serve_forever()
except KeyboardInterrupt:
    print("\nstopped")
