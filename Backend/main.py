from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordRequestForm
from fastapi.middleware.cors import CORSMiddleware
from sqlmodel import select
from sqlmodel.ext.asyncio.session import AsyncSession
from contextlib import asynccontextmanager
from pydantic import BaseModel
from typing import List, Literal
import os
from dotenv import load_dotenv

# --- EXISTING CHAT IMPORTS ---
from agents import Agent, Runner
from llm_config import config, gemini_key, open_router_config
from qdrant.qdrant_retrieval import retrieve_data

# --- AUTH & DB IMPORTS ---
from auth.db import init_db, get_session
from auth.models import User
from auth.schemas import UserCreate, UserRead, Token
from auth.auth import (
    get_password_hash, 
    verify_password, 
    create_access_token, 
    create_refresh_token, 
    get_current_user,
    decode_token
)


# --- STARTUP EVENT ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    await init_db()
    yield

app = FastAPI(lifespan=lifespan)

# --- CORS SETUP ---
# Combine your WEB_URL with localhost for flexibility
origins = [
    "http://localhost:3000",
    "http://127.0.0.1:3000",
]

load_dotenv()

web_url = os.getenv("WEB_URL")
if web_url:
    origins.append(web_url)

print("🌐 CORS allowed origins:", origins)

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ==========================================
# AUTHENTICATION ENDPOINTS (New)
# ==========================================

@app.post("/register", response_model=UserRead)
async def register(user: UserCreate, session: AsyncSession = Depends(get_session)):
    statement = select(User).where(User.email == user.email)
    result = await session.exec(statement)
    existing_user = result.first()
    
    if existing_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    
    hashed_pwd = get_password_hash(user.password)
    
    # --- MAP NEW FIELDS HERE ---
    new_user = User(
        email=user.email, 
        name=user.name,
        software_experience=user.software_experience,
        hardware_experience=user.hardware_experience,
        hashed_password=hashed_pwd
    )
    
    session.add(new_user)
    await session.commit()
    await session.refresh(new_user)
    return new_user

@app.post("/token", response_model=Token)
async def login(form_data: OAuth2PasswordRequestForm = Depends(), session: AsyncSession = Depends(get_session)):
    """
    Login to get Access Token and Refresh Token.
    """
    # OAuth2 spec uses 'username', but we map it to email
    statement = select(User).where(User.email == form_data.username)
    result = await session.exec(statement)
    user = result.first()

    if not user or not verify_password(form_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    # Generate tokens
    access_token = create_access_token(data={"sub": user.email})
    refresh_token = create_refresh_token(data={"sub": user.email})
    
    return {
        "access_token": access_token, 
        "refresh_token": refresh_token, 
        "token_type": "bearer"
    }

@app.post("/refresh", response_model=Token)
async def refresh_token(token_data: dict, session: AsyncSession = Depends(get_session)):
    """
    Use a long-lived refresh token to get a new access token.
    """
    refresh_token_str = token_data.get("refresh_token")
    if not refresh_token_str:
        raise HTTPException(status_code=400, detail="Refresh token missing")

    payload = decode_token(refresh_token_str)
    
    if not payload or payload.get("type") != "refresh":
        raise HTTPException(status_code=401, detail="Invalid refresh token")
    
    email = payload.get("sub")
    statement = select(User).where(User.email == email)
    result = await session.exec(statement)
    user = result.first()
    
    if not user:
        raise HTTPException(status_code=401, detail="User not found")
        
    # Rotate tokens
    return {
        "access_token": create_access_token(data={"sub": user.email}),
        "refresh_token": create_refresh_token(data={"sub": user.email}),
        "token_type": "bearer"
    }

@app.get("/users/me", response_model=UserRead)
async def read_users_me(current_user: User = Depends(get_current_user)):
    """
    Get current logged-in user profile (includes Name).
    """
    return current_user

# ==========================================
# CHATBOT ENDPOINTS (Existing)
# ==========================================

class ChatMessage(BaseModel):
    role: Literal["user", "bot"]
    text: str

class ChatRequest(BaseModel):
    messages: List[ChatMessage]

@app.post("/chat")
async def chat(request: ChatRequest):
    print("📥 Received messages:", request.messages)

    try:
        # 1. Define the Agent
        agent = Agent(
            name="Physical AI & Humanoid Robotics course's Assistant",
            instructions="""
            You are the specialized AI Assistant for the "Physical AI & Humanoid Robotics" Capstone Course. 
            Your goal is to help students understand the curriculum, hardware requirements, and technical concepts of bridging the digital brain with the physical body.

            ### COURSE CONTEXT & KNOWLEDGE BASE:
            
            **Focus:** AI Systems in the Physical World (Embodied Intelligence).
            **Goal:** Applying AI to control Humanoid Robots in simulated and real-world environments using ROS 2, Gazebo, and NVIDIA Isaac.

            **Module Breakdown:**
            1. **The Robotic Nervous System (ROS 2):** Nodes, Topics, Services, rclpy, URDF.
            2. **The Digital Twin (Gazebo & Unity):** Physics simulation, LiDAR/Depth sensors, collision dynamics.
            3. **The AI-Robot Brain (NVIDIA Isaac):** Isaac Sim (Photorealistic sim), Isaac ROS (VSLAM), Nav2 (Path planning).
            4. **Vision-Language-Action (VLA):** OpenAI Whisper (Voice), LLMs for cognitive planning ("Clean the room" -> ROS actions).

            **Hardware Requirements (Critical):**
            - **Sim Rig (Workstation):** Must have NVIDIA RTX 4070 Ti (12GB VRAM) or higher (Ideal: RTX 3090/4090). CPU: i7 13th Gen+. RAM: 64GB DDR5. OS: Ubuntu 22.04 LTS.
            - **Edge AI Kit:** NVIDIA Jetson Orin Nano (8GB) or Orin NX.
            - **Sensors:** Intel RealSense D435i (Vision+Depth), Generic USB IMU, ReSpeaker Mic Array.
            - **Robots:** Unitree Go2 Edu (Quadruped proxy), Unitree G1 (Humanoid), or Hiwonder TonyPi Pro (Budget/Kinematics only).

            **Lab Setup Options:**
            - **On-Prem (High CapEx):** Buying physical PCs and robots.
            - **Cloud/Ether Lab (High OpEx):** AWS g5.2xlarge instances (~$205/quarter) + Local Jetson Kit ($700) for deployment.

            ### ANSWERING GUIDELINES:
            - **Mandatory Tool Use:** Before answering any question related to Humanoid Robotics or Physical AI concepts, you **must call the tool `retrieve_data`** to get relevant content from the course knowledge base.
            - **Tone:** Technical, academic, and helpful. 
            - **Format:** Use Markdown (bold key terms, bullet points for lists).
            - **Scope:** Answer strictly based on the provided course material.
            - **Refusals:** Politely decline to answer off-topic questions (e.g., "What is the weather?", "Tell me a joke").
            - **Hardware Questions:** Be very specific about specs (VRAM, OS versions) as this is a technical bottleneck for the course.
            - **Tool Instructions:** Only use `retrieve_data` to fetch content. After fetching, summarize or reformat as needed to answer the user query. Do not answer without using it first.
            """,
            tools=[retrieve_data]
        )

        print("⚙️ Running agent... ")

        # 2. Prepare the input context
        conversation_context = "\n".join(
            [f"{message.role}: {message.text}" for message in request.messages]
        )

        # 3. Execute using await Runner.run()
        result = await Runner.run(
            agent,
            input=conversation_context,
            run_config=open_router_config,
        )

        print("✅ Response generated successfully")

        # 4. Return the final output directly as JSON
        return {"response": result.final_output}

    except Exception as e:
        print("❌ Error:", str(e))
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/")
async def health():
    return {
        "status": "healthy",
        "response": "api set" if gemini_key else "API key missing",
        "gemini_api_key_set": bool(gemini_key),
        "web_url": os.getenv("WEB_URL", "not set"),
    }