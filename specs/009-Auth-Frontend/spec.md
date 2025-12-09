# Frontend Specification

## 1. Overview
A Docusaurus-based documentation site that includes a custom Authentication layer. It allows users to Sign Up, Login, and access protected content, with the UI adapted to the user's login state.

## 2. Tech Stack
- **Framework:** Docusaurus (React).
- **HTTP Client:** Axios.
- **State Management:** React Context API (`AuthContext`).
- **Styling:** CSS Modules + Infima (Docusaurus Theme Variables).

## 3. Features
- **Global Auth Wrapper:** The entire app is wrapped in an Auth Provider using the `Root` theme component.
- **Auto-Refresh:** Axios interceptors or Context logic to handle 401 errors using Refresh Tokens.
- **Custom Navbar:**
    - *Guest:* Shows "Login" and "Sign Up" buttons.
    - *User:* Shows "Name" with a dropdown for "Dashboard" and "Logout".
- **Registration Form:** Collects Name, Email, Password, Software Exp, Hardware Exp.
- **Theming:** Dark-mode compatible forms using CSS variables (`var(--ifm-color-primary)`, etc.).

## 4. Pages
- `/login`: Login form.
- `/register`: extended registration form with Dropdowns for experience levels.
- `/dashboard`: Protected route example.