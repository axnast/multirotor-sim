import { defineConfig } from "vite";

/**
 * GitHub Pages serves project sites from /<repo>/. Override with VITE_BASE
 * in CI; local development and the preview server use "/".
 */
const base = process.env.VITE_BASE ?? "/";

export default defineConfig({
  base,
  build: {
    target: "es2022",
    outDir: "dist",
  },
  server: {
    port: 5173,
  },
  preview: {
    port: 4173,
  },
});
