import { envelope } from "../lib/http.mjs";

const notFound = (reply) => reply.code(404).send({ error: "Layout not found" });

export const registerLayoutRoutes = (app, { layoutStore }) => {
  app.get("/workspaces/:workspace/layouts", async ({ params }) => {
    const layouts = await layoutStore.listLayouts(params.workspace);
    return envelope(`/workspaces/${params.workspace}/layouts`, layouts);
  });

  app.post("/workspaces/:workspace/layout", async ({ params, body }) => {
    const layout = await layoutStore.createLayout(params.workspace, body);
    return envelope(`/workspaces/${params.workspace}/layouts`, { layout });
  });

  app.put("/layouts/:id", async ({ params, body }, reply) => {
    const layout = await layoutStore.updateLayoutByExternalId(params.id, body);
    return layout ? envelope(`/layouts/${params.id}`, layout) : notFound(reply);
  });

  app.delete("/workspaces/:workspace/layout/:id", async ({ params }, reply) => {
    const layout = await layoutStore.deleteLayout(params.workspace, params.id);
    return layout ? envelope(`/workspaces/${params.workspace}/layout/${params.id}`, layout) : notFound(reply);
  });
};
