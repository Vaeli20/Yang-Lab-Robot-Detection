from fastapi import FastAPI
from fastapi.responses import JSONResponse

app = FastAPI()

is_running = False  # Global state

@app.get("/")
def root():
    return {"status": "online"}

@app.post("/start")
def start_detection():
    global is_running
    is_running = True
    return JSONResponse(content={"message": "Detection started"})

@app.post("/stop")
def stop_detection():
    global is_running
    is_running = False
    return JSONResponse(content={"message": "Detection stopped"})

@app.get("/status")
def get_status():
    return {"running": is_running}
