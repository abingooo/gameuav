#!/usr/bin/env python3

import argparse
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import StreamingResponse

from gateway.camera_stream_gateway.cameras import CameraCaptureError, CameraManager, CameraStreamError
from gateway.camera_stream_gateway.schemas import CameraSettingsRequest


def create_app(workspace_root=None):
    workspace_root = workspace_root or str(Path(__file__).resolve().parents[2])
    app = FastAPI(
        title="GameUAV Camera Stream Gateway",
        version="0.1.0",
        description="UAV-local HTTP/MJPEG gateway for ROS camera topics.",
    )
    app.state.camera_manager = CameraManager(workspace_root=workspace_root)

    @app.get("/health")
    def health():
        return {"ok": True}

    @app.get("/api/cameras/settings")
    def camera_settings(request: Request):
        return {
            "ok": True,
            "cameras": request.app.state.camera_manager.list_settings(),
        }

    @app.get("/api/cameras/stats")
    def camera_stats(request: Request):
        return {
            "ok": True,
            "streams": request.app.state.camera_manager.stream_stats(),
        }

    @app.post("/api/cameras/{camera_id}/settings")
    def camera_settings_update(camera_id: str, request: Request, payload: CameraSettingsRequest):
        update = payload.model_dump(exclude_none=True) if hasattr(payload, "model_dump") else payload.dict(exclude_none=True)
        try:
            settings = request.app.state.camera_manager.update_settings(camera_id, update)
        except KeyError:
            raise HTTPException(status_code=404, detail="unknown camera: %s" % camera_id)
        return {
            "ok": True,
            "camera_id": camera_id,
            "settings": settings,
        }

    @app.get("/api/cameras/{camera_id}/snapshot")
    def camera_snapshot(camera_id: str, request: Request, quality: int = None):
        try:
            frame = request.app.state.camera_manager.capture_snapshot(camera_id, quality=quality)
        except KeyError:
            raise HTTPException(status_code=404, detail="unknown camera: %s" % camera_id)
        except CameraCaptureError as exc:
            raise HTTPException(status_code=503, detail=str(exc))
        return frame

    @app.get("/api/cameras/{camera_id}/stream")
    def camera_stream(camera_id: str, request: Request, quality: int = None, fps: float = 5.0):
        try:
            client_host = request.client.host if request.client else "unknown"
            stream = request.app.state.camera_manager.stream_mjpeg(
                camera_id,
                quality=quality,
                fps=fps,
                stream_owner=client_host,
            )
        except KeyError:
            raise HTTPException(status_code=404, detail="unknown camera: %s" % camera_id)
        except CameraStreamError as exc:
            raise HTTPException(status_code=503, detail=str(exc))
        return StreamingResponse(
            stream,
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={
                "Cache-Control": "no-store",
                "X-Accel-Buffering": "no",
            },
        )

    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run the UAV-local camera stream gateway")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9200)
    parser.add_argument("--workspace-root", default=str(Path(__file__).resolve().parents[2]))
    args = parser.parse_args(argv)

    import uvicorn

    uvicorn.run(
        create_app(args.workspace_root),
        host=args.host,
        port=args.port,
        access_log=False,
        log_level="warning",
    )


if __name__ == "__main__":
    main()
