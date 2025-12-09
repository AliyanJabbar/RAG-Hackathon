# Frontend Tasks Checklist

- [ ] **Setup**
    - [ ] Run: `npm install axios` inside frontend folder.

- [ ] **Context (`src/components/AuthContext.js`)**
    - [ ] Create `AuthProvider`.
    - [ ] Implement `login` (call `/token`).
    - [ ] Implement `refreshAccessToken` (call `/refresh`).
    - [ ] Implement `logout` (clear localStorage).

- [ ] **Global Wrapper (`src/theme/Root.js`)**
    - [ ] **Important:** Ensure file is exactly at `src/theme/Root.js` (Capital R).
    - [ ] Wrap `{children}` with `<AuthProvider>`.

- [ ] **Styling (`src/css/auth.module.css`)**
    - [ ] Define `.authCard`, `.input`, `.submitButton`.
    - [ ] Use `var(--ifm-*)` variables for Dark Mode support.
    - [ ] Add specific styling for `<select>` dropdowns.

- [ ] **Navbar Component (`src/components/AuthNavbarItem.js`)**
    - [ ] Implement logic to toggle between Guest/User view.
    - [ ] Add "Logout" functionality.
    - [ ] Add Dropdown menu structure.

- [ ] **Register Component (`src/theme/NavbarItem/ComponentTypes.js`)**
    - [ ] Add `custom-auth` mapping to `AuthNavbarItem`.

- [ ] **Config (`docusaurus.config.js`)**
    - [ ] Add `{ type: 'custom-auth', position: 'right' }` to navbar items.

- [ ] **Pages**
    - [ ] **`src/pages/login.js`**: Implement form using CSS modules.
    - [ ] **`src/pages/register.js`**:
        - [ ] Add Name input.
        - [ ] Add Software Experience dropdown.
        - [ ] Add Hardware Experience dropdown.
        - [ ] Connect to backend `/register`.