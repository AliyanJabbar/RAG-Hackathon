# Backend Specification

## 1. Overview
A production-ready FastAPI backend serving as an AI Chatbot and Authentication provider. It uses a Serverless Postgres database (Neon) accessed via an asynchronous driver.

## 2. Tech Stack
- **Language:** Python 3.10+
- **Framework:** FastAPI
- **Database ORM:** SQLModel (with SQLAlchemy under the hood)
- **Database Driver:** `asyncpg` (Asynchronous PostgreSQL)
- **Authentication:** OAuth2 with JWT (Access + Refresh Tokens)
- **Security:** `passlib` with `bcrypt` (pinned to v3.2.2 compatibility)

## 3. Database Schema (`User` Table)
The database must support the following columns to handle user profiles and customization levels:
- `id`: Integer, Primary Key.
- `email`: String, Unique, Index.
- `name`: String.
- `software_experience`: String (e.g., "beginner", "intermediate").
- `hardware_experience`: String.
- `hashed_password`: String.
- `is_active`: Boolean.

## 4. Environment Variables
The `.env` file must contain:
- `DATABASE_URL`: Must be formatted as `postgresql+asyncpg://...` and include `ssl=require`.
- `SECRET_KEY`: For JWT signing.
- `ALGORITHM`: `HS256`.
- `WEB_URL`: Allowed CORS origin (e.g., `http://localhost:3000`).

## 5. API Endpoints
- **POST /register**: Accepts Name, Email, Password, Software/Hardware Experience. Returns User Profile.
- **POST /token**: Accepts form-data (username/password). Returns Access & Refresh Tokens.
- **POST /refresh**: Accepts Refresh Token. Returns new Access Token.
- **GET /users/me**: Protected. Returns current user profile.
- **POST /chat**: Protected (Optional) or Public. Runs the AI Agent.