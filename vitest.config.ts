import { defineConfig } from "vitest/config";

export default defineConfig({
  test: {
    environment: "node",
    include: ["tests/**/*.test.ts"],
    // M5's seeded-statistics tests are CPU heavy. Cap workers so pre-existing
    // tests keep their 5 s default timeout on shared/slow CI runners.
    maxWorkers: 2,
  },
});
