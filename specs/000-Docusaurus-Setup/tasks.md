# /sp.plan

## Step 1 — Create the /frontend directory

* Ensure the project root contains a `frontend` folder.
* Navigate into it:

  ```bash
  mkdir frontend
  cd frontend
  ```

## Step 2 — Initialize the Docusaurus project

* Run the official Docusaurus generator:

  ```bash
  npx create-docusaurus@latest docs-site classic
  ```
* This command creates the folder:

  ```
  /frontend/docs-site
  ```

## Step 3 — Install dependencies

* Move into the generated Docusaurus directory:

  ```bash
  cd docs-site
  npm install
  ```

## Step 4 — Verify project structure

Docusaurus automatically creates:

```
docs-site/
 ├── docs/
 ├── blog/
 ├── src/
 │   └── pages/
 ├── docusaurus.config.js
 ├── sidebars.js
 └── package.json
```

## Step 5 — Run the development server

* Start local dev mode:

  ```bash
  npm start
  ```

## Step 6 — Build the production-optimized static site

* Generate the `build` folder:

  ```bash
  npm run build
  ```

## Step 7 — Preview the production build locally

* Serve the static output locally:

  ```bash
  npm run serve
  ```

## Step 8 — Deploy the site

Upload the `build/` directory to any static hosting provider:

* **Vercel**
* **Netlify**
* **GitHub Pages**
* **Cloudflare Pages**
* **Any static file server**
