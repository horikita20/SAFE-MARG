"""
FastAPI Server Entry Point for SIH26037 Smart Autonomous Vehicle.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from api.routes import router

app = FastAPI(
    title="Smart Autonomous Vehicle API (SIH26037)",
    description="Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads",
    version="1.0.0",
)

# CORS middleware for React frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(router, prefix="/api")


@app.get("/")
def root():
    return {
        "project": "SIH26037: Smart Autonomous Vehicle",
        "description": "Adaptive Indian Road Navigation & Collision Avoidance",
        "status": "online",
        "docs_url": "/docs",
    }


if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
