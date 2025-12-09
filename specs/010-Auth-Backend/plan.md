# Backend Implementation Plan

## Phase 1: Environment & Dependencies
1. Initialize project with `uv` or `pip`.
2. strict versioning for `bcrypt` (v3.2.2) to prevent `passlib` conflicts.
3. Install async database drivers (`asyncpg`).

## Phase 2: Database Layer
1. Configure `database.py` to handle the Neon connection string quirks (replacing `postgres://` with `postgresql+asyncpg://` and `sslmode` with `ssl`).
2. Define the `User` model in `models.py` including the new Experience fields.

## Phase 3: Authentication Logic
1. Implement password hashing utilities.
2. Implement JWT generation (short-lived access token, long-lived refresh token).
3. Create dependency `get_current_user` using pure SQLModel syntax (`session.exec`).

## Phase 4: API Development
1. **Register Endpoint:** Logic to check existing email -> Hash Password -> Save new fields (`software_experience`, `hardware_experience`).
2. **Login Endpoint:** Verify credentials -> Issue Token Pair.
3. **Refresh Endpoint:** Validate refresh token -> Rotate tokens.
4. **Chat Endpoint:** Integrate existing AI Agent logic.

## Phase 5: CORS & Deployment
1. Configure CORS to allow requests from Docusaurus (`localhost:3000`).
2. **Critical:** Run DB Reset script or Drop Table via SQL Console to apply new Schema.