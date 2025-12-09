# Frontend Implementation Plan

## Phase 1: Configuration & State
1. Install `axios`.
2. Create `AuthContext.js` to manage `user` state, `login`, `logout`, and token storage (localStorage).
3. Implement `fetchUser` logic to persist login on page refresh.

## Phase 2: Theme Integration (Swizzling)
1. Create `src/theme/Root.js` (or `.tsx`).
2. Wrap the Docusaurus root in `<AuthProvider>`. This ensures `useAuth()` works everywhere.

## Phase 3: Components
1. **Navbar Item:** Create `AuthNavbarItem.js`. Logic: `if (user) show_dropdown else show_buttons`.
2. Register custom navbar type in `src/theme/NavbarItem/ComponentTypes.js`.
3. Update `docusaurus.config.js` to add the item to the nav bar.

## Phase 4: Pages & Styling
1. Create `src/css/auth.module.css` for standardized, dark-mode ready form styling.
2. **Register Page:** Build form with `<select>` inputs for experience levels. Connect to Backend API.
3. **Login Page:** Build standard login form.
4. Apply error handling (display API errors to user).