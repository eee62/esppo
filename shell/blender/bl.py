#!/usr/bin/env python3
"""Send python code to the Blender MCP addon socket (localhost:9876). Usage: bl.py file.py | bl.py -c 'code'"""
import json, socket, sys
code = sys.argv[2] if sys.argv[1] == "-c" else open(sys.argv[1]).read()
s = socket.create_connection(("127.0.0.1", 9876), timeout=3600)
s.sendall(json.dumps({"type": "execute_code", "params": {"code": code}}).encode())
buf = b""
while True:
    ch = s.recv(1 << 20)
    if not ch: break
    buf += ch
    try:
        r = json.loads(buf.decode()); break
    except ValueError:
        continue
if r.get("status") == "success":
    res = r.get("result", {})
    print(res.get("result", res) if isinstance(res, dict) else res)
else:
    print("ERROR:", r.get("message", r)); sys.exit(1)
