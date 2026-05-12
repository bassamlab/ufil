import { promises as fs } from "node:fs";
import path from "node:path";
import { strFromU8, unzipSync } from "fflate";
import { extensionsPath, parseScopedRecordId, scopedRecordId } from "./workspaces.mjs";

const toApiItem = (workspace, folderName, metadata) => {
  const extensionId = metadata.extensionId ?? metadata.id ?? folderName;
  const id = scopedRecordId(workspace, extensionId);
  const name = metadata.name ?? extensionId;

  return {
    id,
    extensionId,
    fileId: metadata.fileId ?? `file_${id}`,
    scope: metadata.scope ?? "org",
    name,
    displayName: metadata.displayName ?? name,
    publisher: metadata.publisher ?? "unknown",
    qualifiedName: metadata.qualifiedName ?? name,
    version: metadata.version ?? "0.0.0",
    description: metadata.description ?? "",
    homepage: metadata.homepage ?? "",
    license: metadata.license ?? "",
    keywords: Array.isArray(metadata.keywords) ? metadata.keywords : [],
    readme: metadata.readme ?? "",
    changelog: metadata.changelog ?? "",
  };
};

const readArchiveEntry = (entries, entryName) => {
  const entry = entries[entryName];
  return entry ? strFromU8(entry) : "";
};

const readArchiveMetadata = async (fsImpl, archivePath, fallbackId) => {
  const archiveEntries = unzipSync(await fsImpl.readFile(archivePath));
  const packageBody = readArchiveEntry(archiveEntries, "package.json");
  if (packageBody.length === 0) {
    throw new Error(`Missing package.json in ${archivePath}`);
  }

  const pkg = JSON.parse(packageBody);
  const name = typeof pkg.name === "string" && pkg.name.trim().length > 0 ? pkg.name.trim() : fallbackId;
  const publisher = typeof pkg.publisher === "string" && pkg.publisher.trim().length > 0 ? pkg.publisher.trim() : "unknown";

  return {
    extensionId: `${publisher}.${name}`,
    scope: "org",
    name,
    displayName: typeof pkg.displayName === "string" && pkg.displayName.trim().length > 0 ? pkg.displayName.trim() : name,
    publisher,
    qualifiedName: typeof pkg.qualifiedName === "string" && pkg.qualifiedName.trim().length > 0 ? pkg.qualifiedName.trim() : name,
    version: typeof pkg.version === "string" && pkg.version.trim().length > 0 ? pkg.version.trim() : "0.0.0",
    description: typeof pkg.description === "string" ? pkg.description : "",
    homepage: typeof pkg.homepage === "string" ? pkg.homepage : "",
    license: typeof pkg.license === "string" ? pkg.license : "",
    keywords: Array.isArray(pkg.keywords) ? pkg.keywords : [],
    readme: readArchiveEntry(archiveEntries, "README.md"),
    changelog: readArchiveEntry(archiveEntries, "CHANGELOG.md"),
  };
};

export const createExtensionStore = ({ workspacesRoot, fsImpl = fs } = {}) => {
  const listExtensions = async (workspace) => {
    const root = extensionsPath(workspacesRoot, workspace);
    await fsImpl.mkdir(root, { recursive: true });
    const entries = await fsImpl.readdir(root, { withFileTypes: true });
    const extensions = [];

    for (const entry of entries) {
      if (!entry.isFile() || !entry.name.endsWith(".foxe")) continue;

      const archivePath = path.join(root, entry.name);
      const fallbackId = entry.name.replace(/\.foxe$/u, "");

      try {
        const metadata = await readArchiveMetadata(fsImpl, archivePath, fallbackId);
        extensions.push(toApiItem(workspace, fallbackId, metadata));
      } catch {
        // Best-effort: skip invalid extension archives.
      }
    }

    return extensions.sort((left, right) => left.id.localeCompare(right.id));
  };

  const getDownload = async (externalId) => {
    const parsed = parseScopedRecordId(externalId);
    if (!parsed) {
      return null;
    }

    const filePath = path.join(extensionsPath(workspacesRoot, parsed.workspace), `${parsed.id}.foxe`);

    try {
      await fsImpl.access(filePath);
      return { filePath, downloadName: `${externalId}.foxe` };
    } catch {
      return null;
    }
  };

  return {
    listExtensions,
    getDownload,
  };
};
