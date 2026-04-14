from __future__ import annotations

from pathlib import Path

import uvicorn
from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .analysis_service import AnalysisRequest, analyze_uploaded_csv

WEB_DIR = Path(__file__).with_name("web")

app = FastAPI(title="PI Notch Analysis")
app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")


def _parse_gains(raw: str) -> list[float]:
    items = [part.strip() for part in raw.split(",")]
    gains = [float(item) for item in items if item]
    if not gains:
        raise ValueError("At least one candidate gain is required")
    return gains


def _parse_bool(raw: str) -> bool:
    value = raw.strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise ValueError("Boolean form value must be true/false")


@app.get("/")
def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")


@app.post("/api/analyze")
async def analyze(
    file: UploadFile = File(...),
    motor: str = Form("a"),
    wheel_diameter: float = Form(0.125),
    kp: float = Form(300.0),
    ki: float = Form(300.0),
    kd: float = Form(0.0),
    settle_time: float = Form(5.0),
    tail_time: float = Form(5.0),
    gains: str = Form("0.01,0.015,0.02,0.025,0.03,0.04,0.05,0.08"),
    speed_notch_enabled: str = Form("false"),
    speed_notch_freq_hz: float = Form(0.66),
    speed_notch_q: float = Form(6.0),
) -> dict[str, object]:
    if motor not in {"a", "b"}:
        raise HTTPException(status_code=400, detail="motor must be 'a' or 'b'")

    try:
        csv_text = (await file.read()).decode("utf-8-sig")
        parsed_gains = _parse_gains(gains)
        parsed_speed_notch_enabled = _parse_bool(speed_notch_enabled)
        return analyze_uploaded_csv(
            AnalysisRequest(
                csv_text=csv_text,
                source_name=file.filename or "uploaded.csv",
                motor=motor,
                wheel_diameter=wheel_diameter,
                kp=kp,
                ki=ki,
                kd=kd,
                settle_time=settle_time,
                tail_time=tail_time,
                gains=parsed_gains,
                speed_notch_enabled=parsed_speed_notch_enabled,
                speed_notch_freq_hz=speed_notch_freq_hz,
                speed_notch_q=speed_notch_q,
            )
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


def main() -> None:
    uvicorn.run(app, host="127.0.0.1", port=8000)


if __name__ == "__main__":
    main()
