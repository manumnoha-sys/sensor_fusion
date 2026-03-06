"""NUC dashboard server for sensor_fusion cameras.

Proxies MJPEG streams from the Kria board and serves a live web dashboard.

Run on the NUC:
    pip install fastapi uvicorn httpx
    python3 app.py

    Or:
    uvicorn app:app --host 0.0.0.0 --port 5000

Dashboard: http://100.83.146.119:5000
"""

import argparse
import logging
import os

import httpx
from fastapi import FastAPI, Response
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger(__name__)

KRIA_BASE = os.getenv("KRIA_URL", "http://100.101.58.66:8080")

app = FastAPI(title="Sensor Fusion Dashboard")


async def _proxy_stream(url: str):
    """Async generator that proxies a remote MJPEG stream."""
    async with httpx.AsyncClient(timeout=None) as client:
        async with client.stream("GET", url) as resp:
            async for chunk in resp.aiter_bytes(chunk_size=4096):
                yield chunk


@app.get("/stream/{idx}")
async def stream(idx: int):
    """Proxy MJPEG stream from Kria camera {idx}."""
    url = f"{KRIA_BASE}/stream/{idx}"
    return StreamingResponse(
        _proxy_stream(url),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/snapshot/{idx}")
async def snapshot(idx: int):
    """Proxy single JPEG snapshot from Kria camera {idx}."""
    async with httpx.AsyncClient(timeout=5) as client:
        try:
            resp = await client.get(f"{KRIA_BASE}/snapshot/{idx}")
            return Response(content=resp.content, media_type="image/jpeg")
        except httpx.RequestError:
            return Response(status_code=503)


@app.get("/cameras")
async def cameras():
    """List available cameras from Kria."""
    async with httpx.AsyncClient(timeout=5) as client:
        try:
            resp = await client.get(f"{KRIA_BASE}/")
            return resp.json()
        except httpx.RequestError:
            return {"error": "Kria board unreachable", "kria_url": KRIA_BASE}


@app.get("/", response_class=HTMLResponse)
async def dashboard():
    with open(os.path.join(os.path.dirname(__file__), "templates", "index.html")) as f:
        return f.read()


if __name__ == "__main__":
    import uvicorn
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    args = parser.parse_args()
    uvicorn.run(app, host=args.host, port=args.port)
