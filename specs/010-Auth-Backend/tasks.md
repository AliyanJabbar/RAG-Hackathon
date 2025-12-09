# Backend Tasks Checklist

- [ ] **Dependency Installation**
    - [ ] Run: `uv add fastapi[standard] sqlmodel asyncpg python-jose[cryptography] python-multipart python-dotenv email-validator`
    - [ ] **Critical Fix:** Run `uv add "bcrypt==3.2.2"` (Fixes "password cannot be longer than 72 bytes" error).

- [ ] **Configuration**
    - [ ] Create `.env` file.
    - [ ] Set `DATABASE_URL` (Ensure `postgresql+asyncpg://` prefix).

- [ ] **Database Setup (`database.py`)**
    - [ ] Implement `create_async_engine`.
    - [ ] Add auto-fix logic for SSL parameters in connection string.

- [ ] **Models (`models.py`)**
    - [ ] Define `User` class.
    - [ ] Add `software_experience` and `hardware_experience` columns.

- [ ] **Schemas (`schemas.py`)**
    - [ ] Create `UserCreate` (includes new fields).
    - [ ] Create `UserRead` (excludes password).
    - [ ] Create `Token` (includes `access_token` and `refresh_token`).

- [ ] **Authentication (`auth.py`)**
    - [ ] Implement `get_password_hash` & `verify_password`.
    - [ ] Implement `create_access_token` & `create_refresh_token`.
    - [ ] Implement `get_current_user` using `session.exec()`.

- [ ] **Main Application (`main.py`)**
    - [ ] Setup CORS.
    - [ ] Implement `/register` route (saving experience levels).
    - [ ] Implement `/token` route.
    - [ ] Implement `/refresh` route.
    - [ ] Integrate Chatbot Agent code.

- [ ] **Schema Migration (CRITICAL)**
    - [ ] Go to Neon Console -> SQL Editor.
    - [ ] Run: `DROP TABLE "user";`.
    - [ ] Restart server to recreate table with new columns.