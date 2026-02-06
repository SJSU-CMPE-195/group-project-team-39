import urllib.request
import sys

URL = "http://127.0.0.1:8080/ping"  # use 127.0.0.1 if network_mode: host

try:
    with urllib.request.urlopen(URL, timeout=3) as r:
        body = r.read().decode("utf-8", errors="replace").strip()
        print(f"[pysvc] GET /ping -> {r.status}, body='{body}'")
    sys.exit(0)
except Exception as e:
    print(f"[pysvc] ping failed: {e}")
    sys.exit(1)
