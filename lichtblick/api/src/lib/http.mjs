export const envelope = (pathValue, data) => ({
  data,
  timestamp: new Date().toISOString(),
  path: pathValue,
});

export const registerHttpBehavior = (app) => {
  app.addHook("onRequest", async (request, reply) => {
    const requestOrigin = request.headers.origin;
    const requestedHeaders = request.headers["access-control-request-headers"];
    const allowHeaders =
      typeof requestedHeaders === "string" && requestedHeaders.trim().length > 0
        ? requestedHeaders
        : "Content-Type, Authorization, Range, baggage, sentry-trace, traceparent";

    reply.header("Access-Control-Allow-Origin", requestOrigin || "*");
    reply.header("Vary", "Origin, Access-Control-Request-Headers");
    reply.header("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE,OPTIONS");
    reply.header("Access-Control-Allow-Headers", allowHeaders);
    reply.header("Access-Control-Expose-Headers", "Content-Disposition, Content-Length");

    if (request.method === "OPTIONS") {
      return reply.code(204).send();
    }
  });
};
