import path from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const defaultWorkspacesRoot = path.resolve(__dirname, "..", "..", "workspaces");

export const getConfig = ({ env = process.env } = {}) => ({
  workspacesRoot: env.LICHTBLICK_WORKSPACES_DIR ?? defaultWorkspacesRoot,
  host: env.LICHTBLICK_API_HOST ?? "0.0.0.0",
  port: Number(env.LICHTBLICK_API_PORT ?? "8766"),
  layoutsUser: env.LICHTBLICK_LAYOUTS_USER ?? "user@example.com",
});
