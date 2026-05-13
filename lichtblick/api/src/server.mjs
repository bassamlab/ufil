#!/usr/bin/env node
import { buildApp } from "./app.mjs";
import { getConfig } from "./config.mjs";

const config = getConfig();
const app = buildApp({ config });

await app.listen({ host: config.host, port: config.port });
