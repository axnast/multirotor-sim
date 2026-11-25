import { defineConfig } from "vitest/config";

/** Heavy seeded-statistics suites: run explicitly with `npm run test:slow`. */
export default defineConfig({
  test: {
    environment: "node",
    include: ["tests/slow/**/*.test.ts"],
    maxWorkers: 2,
  },
});
