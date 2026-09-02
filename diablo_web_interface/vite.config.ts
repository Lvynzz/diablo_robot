import { defineConfig, loadEnv } from "vite";
import react from "@vitejs/plugin-react";
import { resolve } from "node:path";

export default defineConfig(({ command, mode }) => {
  const env = loadEnv(mode, process.cwd(), "");
  const robotUrl = env.VITE_ROBOT_URL || "http://127.0.0.1:8000";

  return {
    plugins: [react()],
    base: command === "build" ? "/static/" : "/",
    server: {
      host: "0.0.0.0",
      port: 3000,
      proxy: {
        "/api": {
          target: robotUrl,
          changeOrigin: true,
        },
        "/ws": {
          target: robotUrl.replace(/^http/, "ws"),
          ws: true,
        },
      },
    },
    build: {
      outDir: resolve(__dirname, "dist"),
      emptyOutDir: true,
    },
  };
});
