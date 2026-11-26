/**
 * Measure real rollout throughput in worker_threads: single-thread steps/s and
 * the speedup from 1..12 workers on this machine. Run with
 * `npm run bench:throughput`. The reported throughput uses the actual shared
 * `simulateStep` core loop, not a synthetic workload.
 */

import { Worker } from "node:worker_threads";

const STEPS_PER_WORKER = 60_000;
const WORKER_COUNTS = [1, 2, 4, 6, 8, 11, 12] as const;

function measure(workers: number): Promise<{ wallMs: number; stepsPerSecond: number }> {
  const start = performance.now();
  const done = Array.from({ length: workers }, () =>
    new Promise<void>((resolve, reject) => {
      const worker = new Worker(new URL("./lib/rolloutWorker.ts", import.meta.url), {
        workerData: STEPS_PER_WORKER,
        execArgv: ["--import", "tsx"],
      });
      worker.on("message", () => resolve());
      worker.on("error", reject);
    }),
  );
  return Promise.all(done).then(() => {
    const wallMs = performance.now() - start;
    return {
      wallMs,
      stepsPerSecond: (workers * STEPS_PER_WORKER) / (wallMs / 1000),
    };
  });
}

async function main(): Promise<void> {
  const single = await measure(1);
  console.log(
    `single worker: ${Math.round(single.stepsPerSecond).toLocaleString()} steps/s ` +
      `(${single.wallMs.toFixed(0)} ms for ${STEPS_PER_WORKER.toLocaleString()} steps)`,
  );
  console.log("workers | steps/s | speedup");
  for (const workers of WORKER_COUNTS) {
    const result = await measure(workers);
    console.log(
      `${String(workers).padStart(7)} | ${Math.round(result.stepsPerSecond).toLocaleString().padStart(8)} | ` +
        `${(result.stepsPerSecond / single.stepsPerSecond).toFixed(2)}x`,
    );
  }
}

main().catch((error: unknown) => {
  console.error(error);
  process.exitCode = 1;
});
