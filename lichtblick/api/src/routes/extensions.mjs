import { createReadStream } from "node:fs";
import { envelope } from "../lib/http.mjs";

export const registerExtensionRoutes = (app, { extensionStore }) => {
  app.get("/workspaces/:workspace/extensions", async ({ params }) => {
    const extensions = await extensionStore.listExtensions(params.workspace);
    return envelope(`workspaces/${params.workspace}/extensions`, extensions);
  });

  app.get("/extensions/:id/download", async ({ params }, reply) => {
    const download = await extensionStore.getDownload(params.id);
    if (!download) {
      return reply.code(404).send({ error: "Extension not found" });
    }

    reply.header("Content-Type", "application/octet-stream");
    reply.header("Content-Disposition", `attachment; filename="${download.downloadName}"`);
    return reply.send(createReadStream(download.filePath));
  });
};
