import path from "node:path";

export const scopedRecordId = (workspace, id) => `${workspace}__${id}`;

export const parseScopedRecordId = (id) => {
  const split = id.indexOf("__");
  return split > 0 ? { workspace: id.slice(0, split), id: id.slice(split + 2) } : null;
};

export const workspacePath = (workspacesRoot, workspace) => path.join(workspacesRoot, workspace);
export const layoutsPath = (workspacesRoot, workspace) => path.join(workspacePath(workspacesRoot, workspace), "layouts");
export const extensionsPath = (workspacesRoot, workspace) => path.join(workspacePath(workspacesRoot, workspace), "extensions");
export const layoutFile = (workspacesRoot, workspace, storageId) => path.join(layoutsPath(workspacesRoot, workspace), `${storageId}.json`);
