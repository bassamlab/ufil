import Fastify from "fastify";
import { getConfig } from "./config.mjs";
import { registerHttpBehavior } from "./lib/http.mjs";
import { createExtensionStore } from "./lib/extension-store.mjs";
import { createLayoutStore } from "./lib/layout-store.mjs";
import { registerExtensionRoutes } from "./routes/extensions.mjs";
import { registerLayoutRoutes } from "./routes/layouts.mjs";

export const buildApp = ({ config = getConfig(), logger = true, fsImpl } = {}) => {
  const app = Fastify({ logger });
  const defaultJsonParser = app.getDefaultJsonParser("ignore", "ignore");

  app.removeContentTypeParser("application/json");
  app.addContentTypeParser("application/json", { parseAs: "string" }, (request, body, done) => {
    if (body.length === 0) {
      done(null, {});
      return;
    }

    defaultJsonParser(request, body, done);
  });

  const layoutStore = createLayoutStore({
    workspacesRoot: config.workspacesRoot,
    layoutsUser: config.layoutsUser,
    fsImpl,
  });
  const extensionStore = createExtensionStore({
    workspacesRoot: config.workspacesRoot,
    fsImpl,
  });

  registerHttpBehavior(app);

  app.get("/healthz", async () => ({ ok: true }));

  registerLayoutRoutes(app, { layoutStore });
  registerExtensionRoutes(app, { extensionStore });

  return app;
};
