import assert from "node:assert/strict";
import { promises as fs } from "node:fs";
import os from "node:os";
import path from "node:path";
import test from "node:test";
import { fileURLToPath } from "node:url";
import { buildApp } from "../src/app.mjs";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const fixtureWorkspacesRoot = path.resolve(__dirname, "..", "..", "workspaces");
const fixtureExtensionsRoot = path.resolve(__dirname, "..", "..", "extensions");
const fixtureExtensionDir = path.join(fixtureExtensionsRoot, "object_list_schema_converter");

const createTestApp = (t, workspacesRoot) => {
  const app = buildApp({
    logger: false,
    config: {
      workspacesRoot,
      layoutsUser: "tester@example.com",
    },
  });

  t.after(async () => {
    await app.close();
  });

  return app;
};

const createTempWorkspacesRoot = async (t) => {
  const root = await fs.mkdtemp(path.join(os.tmpdir(), "lichtblick-api-"));
  t.after(async () => {
    await fs.rm(root, { recursive: true, force: true });
  });
  return root;
};

test("lists layouts from disk and ignores invalid layout files", async (t) => {
  const root = await createTempWorkspacesRoot(t);
  const layoutsDir = path.join(root, "alpha", "layouts");
  await fs.mkdir(layoutsDir, { recursive: true });
  await fs.writeFile(path.join(layoutsDir, "wrapped.json"), JSON.stringify({ name: "Wrapped", layoutId: "wrapped-id", data: { foo: "bar" } }));
  await fs.writeFile(path.join(layoutsDir, "legacy.json"), JSON.stringify({ configById: {} }));
  await fs.writeFile(path.join(layoutsDir, "broken.json"), "{broken");

  const app = createTestApp(t, root);
  const response = await app.inject({ method: "GET", url: "/workspaces/alpha/layouts" });

  assert.equal(response.statusCode, 200);
  assert.deepEqual(response.json().data.map(({ id, layoutId, name }) => ({ id, layoutId, name })), [
    { id: "alpha__legacy", layoutId: "legacy", name: "legacy" },
    { id: "alpha__wrapped", layoutId: "wrapped-id", name: "Wrapped" },
  ]);
});

test("creates a layout through the write-lite layout endpoint", async (t) => {
  const root = await createTempWorkspacesRoot(t);
  const app = createTestApp(t, root);

  const response = await app.inject({
    method: "POST",
    url: "/workspaces/alpha/layout",
    payload: {
      name: "My Layout",
      layoutId: "layout-1",
      data: { foo: "bar" },
    },
  });

  assert.equal(response.statusCode, 200);
  assert.equal(response.json().data.layout.id, "alpha__My_Layout");
  assert.deepEqual(
    JSON.parse(await fs.readFile(path.join(root, "alpha", "layouts", "My_Layout.json"), "utf8")),
    {
      name: "My Layout",
      layoutId: "layout-1",
      data: { foo: "bar" },
    },
  );
});

test("updates a layout through the client compatibility route", async (t) => {
  const root = await createTempWorkspacesRoot(t);
  const layoutsDir = path.join(root, "alpha", "layouts");
  await fs.mkdir(layoutsDir, { recursive: true });
  await fs.writeFile(path.join(layoutsDir, "Base.json"), JSON.stringify({ name: "Base", layoutId: "base-id", data: { foo: 1 } }));

  const app = createTestApp(t, root);
  const response = await app.inject({
    method: "PUT",
    url: "/layouts/alpha__Base",
    payload: {
      name: "Updated",
      layoutId: "updated-id",
      data: { foo: 2 },
      permission: "ORG_WRITE",
    },
  });

  assert.equal(response.statusCode, 200);
  assert.equal(response.json().data.permission, "ORG_WRITE");
  assert.deepEqual(
    JSON.parse(await fs.readFile(path.join(layoutsDir, "Base.json"), "utf8")),
    {
      name: "Updated",
      layoutId: "updated-id",
      data: { foo: 2 },
    },
  );
});

test("preserves stored layout data when renaming without a data payload", async (t) => {
  const root = await createTempWorkspacesRoot(t);
  const layoutsDir = path.join(root, "alpha", "layouts");
  await fs.mkdir(layoutsDir, { recursive: true });
  await fs.writeFile(
    path.join(layoutsDir, "Base.json"),
    JSON.stringify({ name: "Base", layoutId: "base-id", data: { foo: 1, nested: { bar: true } } }),
  );

  const app = createTestApp(t, root);
  const response = await app.inject({
    method: "PUT",
    url: "/layouts/alpha__Base",
    payload: {
      name: "Renamed",
    },
  });

  assert.equal(response.statusCode, 200);
  assert.equal(response.json().data.id, "alpha__Base");
  assert.deepEqual(response.json().data.data, { foo: 1, nested: { bar: true } });
  assert.deepEqual(
    JSON.parse(await fs.readFile(path.join(layoutsDir, "Base.json"), "utf8")),
    {
      name: "Renamed",
      layoutId: "base-id",
      data: { foo: 1, nested: { bar: true } },
    },
  );
});

test("supports the current client compatibility alias for layout update", async (t) => {
  const root = await createTempWorkspacesRoot(t);
  const layoutsDir = path.join(root, "alpha", "layouts");
  await fs.mkdir(layoutsDir, { recursive: true });
  await fs.writeFile(path.join(layoutsDir, "Compat.json"), JSON.stringify({ name: "Compat", layoutId: "compat-id", data: { value: 1 } }));

  const app = createTestApp(t, root);
  const updateResponse = await app.inject({
    method: "PUT",
    url: "/layouts/alpha__Compat",
    payload: {
      name: "Compat Updated",
      data: { value: 2 },
    },
  });

  assert.equal(updateResponse.statusCode, 200);
  assert.equal(updateResponse.json().data.id, "alpha__Compat");
});

test("deletes a layout through the client workspace route", async (t) => {
  const root = await createTempWorkspacesRoot(t);
  const layoutsDir = path.join(root, "alpha", "layouts");
  await fs.mkdir(layoutsDir, { recursive: true });
  await fs.writeFile(path.join(layoutsDir, "DeleteMe.json"), JSON.stringify({ name: "Delete Me", layoutId: "delete-id", data: { value: 1 } }));

  const app = createTestApp(t, root);
  const response = await app.inject({
    method: "DELETE",
    url: "/workspaces/alpha/layout/alpha__DeleteMe",
  });

  assert.equal(response.statusCode, 200);
  assert.equal(response.json().data.id, "alpha__DeleteMe");
  await assert.rejects(fs.access(path.join(layoutsDir, "DeleteMe.json")));
});

test("accepts delete requests with an empty json body", async (t) => {
  const root = await createTempWorkspacesRoot(t);
  const layoutsDir = path.join(root, "alpha", "layouts");
  await fs.mkdir(layoutsDir, { recursive: true });
  await fs.writeFile(path.join(layoutsDir, "DeleteMe.json"), JSON.stringify({ name: "Delete Me", layoutId: "delete-id", data: { value: 1 } }));

  const app = createTestApp(t, root);
  const response = await app.inject({
    method: "DELETE",
    url: "/workspaces/alpha/layout/alpha__DeleteMe",
    headers: {
      "content-type": "application/json",
    },
    payload: "",
  });

  assert.equal(response.statusCode, 200);
  assert.equal(response.json().data.id, "alpha__DeleteMe");
  await assert.rejects(fs.access(path.join(layoutsDir, "DeleteMe.json")));
});

test("supports the current client workspace delete alias for scoped layout ids", async (t) => {
  const root = await createTempWorkspacesRoot(t);
  const layoutsDir = path.join(root, "alpha", "layouts");
  await fs.mkdir(layoutsDir, { recursive: true });
  await fs.writeFile(path.join(layoutsDir, "CompatDelete.json"), JSON.stringify({ name: "Compat Delete", layoutId: "compat-delete-id", data: { value: 1 } }));

  const app = createTestApp(t, root);
  const response = await app.inject({
    method: "DELETE",
    url: "/workspaces/alpha/layout/alpha__CompatDelete",
  });

  assert.equal(response.statusCode, 200);
  assert.equal(response.json().data.id, "alpha__CompatDelete");
  await assert.rejects(fs.access(path.join(layoutsDir, "CompatDelete.json")));
});

test("lists repo-managed extensions from the fixture workspace", async (t) => {
  const app = createTestApp(t, fixtureWorkspacesRoot);
  const extensionPackage = JSON.parse(await fs.readFile(path.join(fixtureExtensionDir, "package.json"), "utf8"));
  const extensionId = `${extensionPackage.publisher}.${extensionPackage.name}`;
  const readme = await fs.readFile(path.join(fixtureExtensionDir, "README.md"), "utf8");
  const changelog = await fs.readFile(path.join(fixtureExtensionDir, "CHANGELOG.md"), "utf8");
  const response = await app.inject({ method: "GET", url: "/workspaces/ufil/extensions" });

  assert.equal(response.statusCode, 200);
  assert.deepEqual(response.json().data, [
    {
      id: `ufil__${extensionId}`,
      extensionId,
      fileId: `file_ufil__${extensionId}`,
      scope: "org",
      name: extensionPackage.name,
      displayName: extensionPackage.displayName,
      publisher: extensionPackage.publisher,
      qualifiedName: extensionPackage.name,
      version: extensionPackage.version,
      description: extensionPackage.description,
      homepage: extensionPackage.homepage,
      license: extensionPackage.license,
      keywords: extensionPackage.keywords,
      readme,
      changelog,
    },
  ]);
});

test("downloads repo-managed extension content", async (t) => {
  const app = createTestApp(t, fixtureWorkspacesRoot);
  const extensionPackage = JSON.parse(await fs.readFile(path.join(fixtureExtensionDir, "package.json"), "utf8"));
  const extensionId = `${extensionPackage.publisher}.${extensionPackage.name}`;
  const expected = await fs.readFile(
    path.join(fixtureWorkspacesRoot, "ufil", "extensions", `${extensionId}.foxe`),
  );

  const response = await app.inject({
    method: "GET",
    url: `/extensions/ufil__${extensionId}/download`,
  });

  assert.equal(response.statusCode, 200);
  assert.equal(response.headers["content-type"], "application/octet-stream");
  assert.deepEqual(response.rawPayload, expected);
});

test("removed write-heavy routes are not registered", async (t) => {
  const root = await createTempWorkspacesRoot(t);
  const app = createTestApp(t, root);

  const responses = await Promise.all([
    app.inject({ method: "GET", url: "/workspaces/alpha/layouts/layout-1" }),
    app.inject({ method: "POST", url: "/workspaces/alpha/layouts", payload: {} }),
    app.inject({ method: "PUT", url: "/workspaces/alpha/layouts/layout-1", payload: {} }),
    app.inject({ method: "POST", url: "/workspaces/alpha/extension", payload: {} }),
    app.inject({ method: "DELETE", url: "/workspaces/alpha/extension/example" }),
    app.inject({ method: "DELETE", url: "/workspaces/alpha/layouts/layout-1" }),
    app.inject({ method: "DELETE", url: "/layouts/alpha__layout-1" }),
  ]);

  assert.deepEqual(
    responses.map((response) => response.statusCode),
    [404, 404, 404, 404, 404, 404, 404],
  );
});
