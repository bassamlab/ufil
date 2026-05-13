import { promises as fs } from "node:fs";
import path from "node:path";
import { layoutFile, layoutsPath, parseScopedRecordId, scopedRecordId } from "./workspaces.mjs";

const sanitizeLayoutKey = (value) =>
  String(value ?? "")
    .trim()
    .replace(/[^a-zA-Z0-9._-]+/g, "_")
    .replace(/^_+|_+$/g, "")
    .slice(0, 120);

const normalizeLayoutTarget = (workspace, id) => {
  const parsed = parseScopedRecordId(id);
  if (!parsed) {
    return { workspace, storageId: id };
  }

  return parsed.workspace === workspace ? { workspace, storageId: parsed.id } : null;
};

const toLayoutRecord = ({ workspace, storageId, name, layoutId, data, permission = "ORG_READ", layoutsUser }) => ({
  layoutId,
  id: scopedRecordId(workspace, storageId),
  name,
  data,
  workspace,
  permission,
  from: "USER",
  createdBy: layoutsUser,
  updatedBy: layoutsUser,
});

const parseStoredLayout = (storageId, fileBody) => {
  if (
    fileBody &&
    typeof fileBody === "object" &&
    !Array.isArray(fileBody) &&
    typeof fileBody.data === "object" &&
    fileBody.data != null
  ) {
    const name = typeof fileBody.name === "string" && fileBody.name.trim().length > 0 ? fileBody.name.trim() : storageId;
    const layoutId =
      typeof fileBody.layoutId === "string" && fileBody.layoutId.trim().length > 0 ? fileBody.layoutId.trim() : storageId;
    return { name, layoutId, data: fileBody.data };
  }

  return { name: storageId, layoutId: storageId, data: fileBody ?? {} };
};

const writeStoredLayout = async (fsImpl, filePath, layout) => {
  await fsImpl.writeFile(filePath, `${JSON.stringify(layout, null, 2)}\n`, "utf8");
};

const readStoredLayout = async (fsImpl, workspacesRoot, workspace, storageId) => {
  try {
    const body = JSON.parse(await fsImpl.readFile(layoutFile(workspacesRoot, workspace, storageId), "utf8"));
    return { storageId, ...parseStoredLayout(storageId, body) };
  } catch {
    return null;
  }
};

export const createLayoutStore = ({ workspacesRoot, layoutsUser, fsImpl = fs } = {}) => {
  const listLayouts = async (workspace) => {
    const root = layoutsPath(workspacesRoot, workspace);
    await fsImpl.mkdir(root, { recursive: true });
    const entries = await fsImpl.readdir(root, { withFileTypes: true });
    const layouts = [];

    for (const entry of entries) {
      if (!entry.isFile() || !entry.name.endsWith(".json")) continue;

      const storageId = entry.name.replace(/\.json$/u, "");
      try {
        const body = JSON.parse(await fsImpl.readFile(path.join(root, entry.name), "utf8"));
        const { name, layoutId, data } = parseStoredLayout(storageId, body);
        layouts.push(toLayoutRecord({ workspace, storageId, name, layoutId, data, layoutsUser }));
      } catch {
        // Best-effort: skip invalid layout json.
      }
    }

    return layouts.sort((left, right) => left.layoutId.localeCompare(right.layoutId));
  };

  const createLayout = async (workspace, body) => {
    const payload = typeof body === "object" && body != null ? body : {};
    const layoutId = String(payload.layoutId ?? payload.id ?? payload.name ?? `layout-${Date.now()}`).trim() || `layout-${Date.now()}`;
    const name = String(payload.name ?? layoutId).trim() || layoutId;
    const storageId = sanitizeLayoutKey(name) || sanitizeLayoutKey(layoutId) || `layout_${Date.now()}`;
    const data = payload.data ?? payload.layout ?? {};

    await fsImpl.mkdir(layoutsPath(workspacesRoot, workspace), { recursive: true });
    await writeStoredLayout(fsImpl, layoutFile(workspacesRoot, workspace, storageId), { name, layoutId, data });

    return toLayoutRecord({ workspace, storageId, name, layoutId, data, layoutsUser });
  };

  const updateLayout = async (workspace, id, body) => {
    const targetRef = normalizeLayoutTarget(workspace, id);
    if (!targetRef) {
      return null;
    }

    const target = await readStoredLayout(fsImpl, workspacesRoot, targetRef.workspace, targetRef.storageId);
    if (!target) {
      return null;
    }

    const payload = typeof body === "object" && body != null ? body : {};
    const data = Object.hasOwn(payload, "data") ? payload.data : target.data;
    const name = String(payload.name ?? target.name).trim() || target.name;
    const layoutId = String(payload.layoutId ?? target.layoutId).trim() || target.layoutId;

    await fsImpl.mkdir(layoutsPath(workspacesRoot, targetRef.workspace), { recursive: true });
    await writeStoredLayout(fsImpl, layoutFile(workspacesRoot, targetRef.workspace, target.storageId), { name, layoutId, data });

    return toLayoutRecord({
      workspace: targetRef.workspace,
      storageId: target.storageId,
      name,
      layoutId,
      data,
      permission: payload.permission ?? "ORG_READ",
      layoutsUser,
    });
  };

  const updateLayoutByExternalId = async (externalId, body) => {
    const parsed = parseScopedRecordId(externalId);
    return parsed ? updateLayout(parsed.workspace, externalId, body) : null;
  };

  const deleteLayout = async (workspace, externalId) => {
    const targetRef = normalizeLayoutTarget(workspace, externalId);
    if (!targetRef) {
      return null;
    }

    const target = await readStoredLayout(fsImpl, workspacesRoot, targetRef.workspace, targetRef.storageId);
    if (!target) {
      return null;
    }

    try {
      await fsImpl.unlink(layoutFile(workspacesRoot, targetRef.workspace, target.storageId));
    } catch {
      return null;
    }

    return toLayoutRecord({
      workspace: targetRef.workspace,
      storageId: target.storageId,
      name: target.name,
      layoutId: target.layoutId,
      data: target.data,
      layoutsUser,
    });
  };

  return {
    listLayouts,
    createLayout,
    updateLayout,
    updateLayoutByExternalId,
    deleteLayout,
  };
};
