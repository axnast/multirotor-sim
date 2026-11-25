import { defineConfig } from "vitest/config";

export default defineConfig({
  test: {
    environment: "node",
    include: ["tests/**/*.test.ts"],
    // CPU-heavy statistical tests live in tests/slow and run via test:slow.
    exclude: ["**/node_modules/**", "**/dist/**", "tests/slow/**"],
    // Cap workers so tests keep their 5 s default timeout on slow CI runners.
    maxWorkers: 2,
  },
});
