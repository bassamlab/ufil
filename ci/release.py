#!/usr/bin/env python3
# Copyright 2026 UniBw M
"""Release orchestrator for UFIL.

Adapted from the CPM Lab release (ros2-workspace-lab/ci/release.py). Releases all UFIL
repositories in the right order:

1. Components: the submodules of the workspace and, before them, their own submodules on
   this GitLab (ufil-object-tracking in ufil-core). External submodules (e.g. ros-carla-msgs
   from GitHub) are not released. Each component finalises its changelogs and package
   versions on dev, points its submodules to the released commits, merges dev into main
   and is tagged, and its GitLab release is created.
2. Workspace: point the submodules to the released component commits, finalise the
   changelog, merge, tag and create its GitLab release.
3. Documentation: bump the release in src/conf.py, merge, tag and create its GitLab release.
4. GitHub: the tag pipelines of the workspace and the documentation publish a flattened
   snapshot to the GitHub mirrors (sync-github-mirror). The release waits for them, so it
   only succeeds once everything is on GitHub.

Every repo goes through the same steps: a merge request from dev into main is merged, and
the dev commit that was merged is tagged. main then has exactly the files of that commit
(checked before tagging), and the tag is on both branches, so `git describe` works on dev
and main.

Commands:
    check     Validate everything and print the plan. Changes nothing.
    release   All steps. Run from the workspace with RELEASE_VERSION set.

Every command is idempotent: if a run fails, fix the reported problem and run it again.
Repos that are already released (tag exists on main) are skipped; a missing GitLab release
is created. Tags are never moved or overwritten.

Changelogs: each package's CHANGELOG.rst (and the workspace's root CHANGELOG.rst) must start
with either a non-empty "Forthcoming" section (renamed to "<version> (<date>)" by the
release) or a non-empty "<version> (<date>)" section. Unchanged packages get "* No changes".
The package.xml versions and the release of the documentation (src/conf.py) are bumped
automatically.

Environment:
    RELEASE_TOKEN    GitLab token (group access token, Maintainer, api scope).
    RELEASE_VERSION  Version to release, e.g. 0.3.0.
    CI_SERVER_URL, CI_PROJECT_PATH, CI_PIPELINE_ID  Provided by GitLab CI.
    PIPELINE_TIMEOUT  Optional override in seconds.
"""

from __future__ import annotations

import argparse
import contextlib
import datetime
import json
import os
import re
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from pathlib import PurePosixPath
from typing import Optional

DEFAULT_SERVER = 'https://git.unibw.de'
DEFAULT_WORKSPACE = 'cas/project/simon/software/ufil/ufil-workspaces/ufil-workspace'
DOCS_REPOS = ['cas/project/simon/software/documentation']
DOCS_CONF = 'src/conf.py'
DOCS_URL = 'https://bassamlab.github.io/ufil-docs/'
# Repos whose tag pipeline publishes to GitHub (sync-github-mirror), and the CI/CD variables
# that job needs unless .gitlab-ci.yml defines them.
MIRROR_VARIABLES = ('GITHUB_MIRROR_DEPLOY_KEY', 'GITHUB_MIRROR_URL')
# Vendored third-party packages with their own versions (ufil-examples/uofa/sbg_driver); the
# release leaves their package.xml and CHANGELOG.rst as they are.
EXTERNAL_PACKAGES = {'sbg_driver'}

DEV = 'dev'
MAIN = 'main'
VERSION_RE = re.compile(r'^\d+\.\d+\.\d+$')


class ReleaseError(Exception):
    """A problem that stops the release. The message is shown to the user."""


# --------------------------------------------------------------------------------------
# Changelog and package.xml handling (pure functions)
# --------------------------------------------------------------------------------------

@dataclass
class Section:
    title: str
    line: int  # index of the title line
    bullets: list


def parse_top_section(text: str) -> Optional[Section]:
    """Return the first '-'-underlined section of a ROS CHANGELOG.rst."""
    lines = text.splitlines()
    heading = None
    for i in range(len(lines) - 1):
        title, underline = lines[i].strip(), lines[i + 1].strip()
        if title and not title.startswith('*') and re.fullmatch(r'-{3,}', underline):
            if heading is None:
                heading = i
            else:
                return Section(lines[heading].strip(), heading,
                               _bullets(lines[heading + 2:i]))
    if heading is None:
        return None
    return Section(lines[heading].strip(), heading, _bullets(lines[heading + 2:]))


def _bullets(lines):
    return [line.strip()[1:].strip() for line in lines if line.strip().startswith('*')]


def version_tuple(version: str):
    return tuple(int(part) for part in version.split('.'))


def section_titles(text: str):
    lines = text.splitlines()
    return [lines[i].strip() for i in range(len(lines) - 1)
            if lines[i].strip() and not lines[i].strip().startswith('*')
            and re.fullmatch(r'-{3,}', lines[i + 1].strip())]


def check_changelog(text: Optional[str], version: str, changed: bool = True):
    """Return ('ok' | 'rename' | 'no_changes' | 'error', message).

    changed tells whether the package has changes since the last release. Unchanged
    packages without an entry get '* No changes'; changed ones must have an entry.
    """
    if text is None:
        return 'error', 'CHANGELOG.rst is missing'
    section = parse_top_section(text)
    if section is None:
        return 'error', 'CHANGELOG.rst has no version sections'
    if section.title == 'Forthcoming':
        if not section.bullets:
            return 'error', "the 'Forthcoming' section is empty"
        if any(title.startswith(version + ' ') for title in section_titles(text)):
            return 'error', (f"has a 'Forthcoming' section and an existing {version} entry; "
                             f'move the entries into {version} or release a newer version')
        return 'rename', f"'Forthcoming' becomes '{version}'"
    match = re.fullmatch(r'(\d+\.\d+\.\d+) \((\d{4}-\d{2}-\d{2})\)', section.title)
    if not match:
        return 'error', f"unexpected top section '{section.title}'"
    top = match.group(1)
    if top == version:
        if not section.bullets:
            return 'error', f"the '{version}' section is empty"
        return 'ok', f'has a {version} entry'
    if version_tuple(top) > version_tuple(version):
        return 'error', f'top entry {top} is newer than {version}'
    if not changed:
        return 'no_changes', "unchanged since the last release, adds '* No changes'"
    return 'error', (f"changed since the last release but has no entry for {version} "
                     f"(top entry is {top}); add a 'Forthcoming' section")


def add_no_changes(text: str, version: str, date: str) -> str:
    section = parse_top_section(text)
    lines = text.splitlines(keepends=True)
    title = f'{version} ({date})'
    lines[section.line:section.line] = [title + '\n', '-' * len(title) + '\n', '* No changes\n', '\n']
    return ''.join(lines)


def rename_forthcoming(text: str, version: str, date: str) -> str:
    section = parse_top_section(text)
    assert section and section.title == 'Forthcoming'
    lines = text.splitlines(keepends=True)
    title = f'{version} ({date})'
    ending = '\n' if lines[section.line].endswith('\n') else ''
    lines[section.line] = title + ending
    ending = '\n' if lines[section.line + 1].endswith('\n') else ''
    lines[section.line + 1] = '-' * len(title) + ending
    return ''.join(lines)


def section_bullets(text: str, version: str):
    """Bullets of the section for version, if it is the top section."""
    section = parse_top_section(text or '')
    if section and (section.title == 'Forthcoming' or section.title.startswith(version + ' ')):
        return section.bullets
    return ['No changes']


PACKAGE_VERSION_RE = re.compile(r'(<version>)\s*([^<\s]+)\s*(</version>)')


def package_version(xml: str) -> Optional[str]:
    match = PACKAGE_VERSION_RE.search(xml)
    return match.group(2) if match else None


def set_package_version(xml: str, version: str) -> str:
    return PACKAGE_VERSION_RE.sub(lambda m: f'{m.group(1)}{version}{m.group(3)}', xml, count=1)


DOCS_RELEASE_RE = re.compile(r"^release = '([^']*)'$", re.M)


def docs_release(conf: str) -> Optional[str]:
    """The release a Sphinx conf.py describes (release = '0.3.0')."""
    match = DOCS_RELEASE_RE.search(conf)
    return match.group(1) if match else None


def set_docs_release(conf: str, version: str) -> str:
    return DOCS_RELEASE_RE.sub(f"release = '{version}'", conf, count=1)


def package_name(xml: str) -> Optional[str]:
    match = re.search(r'<name>\s*([^<\s]+)\s*</name>', xml)
    return match.group(1) if match else None


def parse_gitmodules(text: str, workspace_path: str, host: Optional[str] = None):
    """Return [(submodule path, project path)] from a .gitmodules file.

    With host, submodules on other servers (e.g. GitHub) are left out.
    """
    result, path, url = [], None, None
    for line in text.splitlines() + ['[end]']:
        line = line.strip()
        if line.startswith('['):
            if path and url and (host is None or url_host(url) in (None, host)):
                result.append((path, url_to_project(url, workspace_path)))
            path = url = None
        elif '=' in line:
            key, value = (part.strip() for part in line.split('=', 1))
            if key == 'path':
                path = value
            elif key == 'url':
                url = value
    return result


def url_host(url: str) -> Optional[str]:
    """Host of a submodule URL, None for relative URLs."""
    if url.startswith('../') or url.startswith('./'):
        return None
    match = re.match(r'^[\w.-]+@([\w.-]+):(?!\d+/)', url)  # git@host:group/project.git
    if match:
        return match.group(1)
    return urllib.parse.urlparse(url).hostname


def url_to_project(url: str, workspace_path: str) -> str:
    if url.startswith('../') or url.startswith('./'):
        project = str(PurePosixPath(workspace_path) / url)
        parts = []
        for part in project.split('/'):
            if part == '..':
                parts.pop()
            elif part not in ('', '.'):
                parts.append(part)
        project = '/'.join(parts)
    elif re.match(r'^[\w.-]+@[\w.-]+:', url):  # git@host:group/project.git
        project = url.split(':', 1)[1]
    else:  # https://host/group/project.git or ssh://git@host/group/project.git
        project = urllib.parse.urlparse(url).path
    project = project.strip('/')
    return project[:-4] if project.endswith('.git') else project


# --------------------------------------------------------------------------------------
# GitLab access
# --------------------------------------------------------------------------------------

class NotFound(Exception):
    pass


class ApiError(ReleaseError):
    pass


class GitLab:
    """Small GitLab REST client with the operations the release needs."""

    def __init__(self, server: str, token: str):
        self.server = server.rstrip('/')
        self.host = urllib.parse.urlparse(self.server).hostname
        self.api = self.server + '/api/v4'
        self.token = token

    # -- transport ---------------------------------------------------------------------

    def _request(self, method, path, params=None, body=None, raw=False):
        url = self.api + path
        if params:
            url += '?' + urllib.parse.urlencode(params, doseq=True)
        data = json.dumps(body).encode() if body is not None else None
        headers = {'PRIVATE-TOKEN': self.token, 'Content-Type': 'application/json'}
        attempts = 4 if method == 'GET' else 1
        for attempt in range(attempts):
            request = urllib.request.Request(url, data=data, method=method, headers=headers)
            try:
                with urllib.request.urlopen(request, timeout=120) as response:
                    payload = response.read()
                    next_page = response.headers.get('X-Next-Page')
                break
            except urllib.error.HTTPError as error:
                # Error pages (e.g. of the university firewall) can be long HTML documents.
                text = error.read().decode(errors='replace')[:300]
                if error.code == 404:
                    raise NotFound(f'{method} {path}: {text}') from None
                if error.code >= 500 and attempt + 1 < attempts:
                    print(f'   retrying {method} {path} after HTTP {error.code}', file=sys.stderr, flush=True)
                    time.sleep(5 * (attempt + 1))
                    continue
                raise ApiError(f'GitLab API {method} {path} failed ({error.code}): {text}') from None
            except (urllib.error.URLError, TimeoutError, ConnectionError) as error:
                if attempt + 1 < attempts:
                    print(f'   retrying {method} {path} after {error!r}', file=sys.stderr, flush=True)
                    time.sleep(5 * (attempt + 1))
                    continue
                raise ApiError(f'GitLab API {method} {path} failed: {error}') from None
        if raw:
            return payload.decode(), next_page
        return (json.loads(payload) if payload else None), next_page

    # endpoint is positional-only so that a 'path' query parameter (tree API) does not clash.
    def get(self, endpoint, /, **params):
        return self._request('GET', endpoint, params)[0]

    def paginate(self, endpoint, /, **params):
        params = dict(params, per_page=100, page=1)
        while True:
            items, next_page = self._request('GET', endpoint, params)
            yield from items
            if not next_page:
                return
            params['page'] = next_page

    @staticmethod
    def _p(project):
        return '/projects/' + urllib.parse.quote(project, safe='')

    @staticmethod
    def _q(value):
        return urllib.parse.quote(value, safe='')

    # -- project -----------------------------------------------------------------------

    def is_archived(self, project):
        return self.get(self._p(project))['archived']

    def variable_keys(self, project):
        """Keys of the project's CI/CD variables (the values are not read)."""
        return {variable['key'] for variable in self.paginate(f'{self._p(project)}/variables')}

    # -- repository --------------------------------------------------------------------

    def branch_sha(self, project, branch):
        try:
            return self.get(f'{self._p(project)}/repository/branches/{self._q(branch)}')['commit']['id']
        except NotFound:
            return None

    def tag_sha(self, project, tag):
        try:
            return self.get(f'{self._p(project)}/repository/tags/{self._q(tag)}')['commit']['id']
        except NotFound:
            return None

    def is_ancestor(self, project, ancestor, ref):
        base = self.get(f'{self._p(project)}/repository/merge_base',
                        **{'refs[]': [ancestor, ref]})
        return base['id'] == ancestor

    def commits_ahead(self, project, base, head):
        """Number of commits in head that are not in base."""
        return len(self.get(f'{self._p(project)}/repository/compare', **{'from': base, 'to': head})['commits'])

    def same_files(self, project, a, b):
        """True if commits a and b have identical files (submodule pointers included)."""
        result = self.get(f'{self._p(project)}/repository/compare',
                          **{'from': a, 'to': b, 'straight': 'true'})
        return not result['diffs']

    def changed_on(self, project, base, head):
        """Files that head changed since its merge base with base."""
        diffs = self.get(f'{self._p(project)}/repository/compare', **{'from': base, 'to': head})['diffs']
        return [diff['new_path'] for diff in diffs]

    def read_file(self, project, path, ref):
        try:
            return self._request('GET', f'{self._p(project)}/repository/files/{self._q(path)}/raw',
                                 {'ref': ref}, raw=True)[0]
        except NotFound:
            return None

    def tree(self, project, ref, path='', recursive=False):
        params = {'ref': ref, 'recursive': str(recursive).lower()}
        if path:
            params['path'] = path
        try:
            return list(self.paginate(f'{self._p(project)}/repository/tree', **params))
        except NotFound:
            return []

    def commit_files(self, project, branch, message, files, author):
        body = {
            'branch': branch,
            'commit_message': message,
            'actions': [{'action': 'update', 'file_path': path, 'content': content}
                        for path, content in sorted(files.items())],
        }
        if author[0]:
            body['author_name'], body['author_email'] = author
        return self._request('POST', f'{self._p(project)}/repository/commits', body=body)[0]['id']

    @contextlib.contextmanager
    def checkout(self, project, branch, shallow=False):
        parsed = urllib.parse.urlparse(self.server)
        url = f'{parsed.scheme}://oauth2:{self.token}@{parsed.netloc}{parsed.path}/{project}.git'
        with tempfile.TemporaryDirectory() as directory:
            def git(*args):
                result = subprocess.run(['git', *args], cwd=directory, capture_output=True, text=True)
                if result.returncode != 0:
                    output = (result.stdout + result.stderr).replace(self.token, '***')
                    raise ReleaseError(f"git {args[0]} failed in {project}:\n{output}")
                return result.stdout.strip()
            depth = ('--depth', '1') if shallow else ()
            git('clone', '--quiet', *depth, '--branch', branch, '--no-recurse-submodules', url, '.')
            yield directory, git

    def commit_workspace(self, project, branch, message, files, submodules, author):
        """Commit files and submodule pointers with git (the API cannot update submodules)."""
        with self.checkout(project, branch, shallow=True) as (directory, git):
            for path, content in files.items():
                with open(os.path.join(directory, path), 'w', encoding='utf-8') as handle:
                    handle.write(content)
                git('add', path)
            for path, sha in submodules.items():
                git('update-index', '--cacheinfo', f'160000,{sha},{path}')
            if subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=directory).returncode == 0:
                return None
            name, email = author
            git('-c', f'user.name={name or "UFIL Release"}',
                '-c', f'user.email={email or "noreply@git.unibw.de"}',
                'commit', '--quiet', '-m', message)
            git('push', '--quiet', 'origin', f'HEAD:{branch}')
            return git('rev-parse', 'HEAD')

    def create_tag(self, project, tag, ref, message):
        self._request('POST', f'{self._p(project)}/repository/tags',
                      body={'tag_name': tag, 'ref': ref, 'message': message})

    def release_exists(self, project, tag):
        try:
            self.get(f'{self._p(project)}/releases/{self._q(tag)}')
            return True
        except NotFound:
            return False

    def create_release(self, project, tag, name, description):
        self._request('POST', f'{self._p(project)}/releases',
                      body={'tag_name': tag, 'name': name, 'description': description})

    # -- pipelines ---------------------------------------------------------------------

    def latest_pipeline(self, project, ref, sha, exclude=None, skip_releases=False):
        """(status, web_url) of the newest pipeline of sha on ref. exclude is a pipeline id to
        ignore; skip_releases ignores the pipelines of the release job (they set RELEASE_VERSION),
        e.g. a failed dry run."""
        pipelines = self.get(f'{self._p(project)}/pipelines', ref=ref, sha=sha,
                             order_by='id', sort='desc', per_page=20)
        for pipeline in pipelines:
            if exclude is not None and str(pipeline['id']) == str(exclude):
                continue
            if skip_releases and any(
                    variable['key'] == 'RELEASE_VERSION'
                    for variable in self.get(f'{self._p(project)}/pipelines/{pipeline["id"]}/variables')):
                continue
            return pipeline['status'], pipeline['web_url']
        return None

    # -- merge requests ----------------------------------------------------------------

    def find_mr(self, project, source, target):
        mrs = self.get(f'{self._p(project)}/merge_requests', state='opened',
                       source_branch=source, target_branch=target)
        return mrs[0] if mrs else None

    def create_mr(self, project, source, target, title, description):
        return self._request('POST', f'{self._p(project)}/merge_requests', body={
            'source_branch': source, 'target_branch': target, 'title': title,
            'description': description, 'remove_source_branch': False,
        })[0]

    def mr_status(self, project, iid, timeout=180):
        """Return the detailed merge status once GitLab has computed it."""
        deadline = time.monotonic() + timeout
        recheck = True
        while True:
            params = {'with_merge_status_recheck': 'true'} if recheck else {}
            mr = self.get(f'{self._p(project)}/merge_requests/{iid}', **params)
            recheck = False
            status = mr.get('detailed_merge_status')
            if status is None:  # GitLab < 15.6
                status = {'can_be_merged': 'mergeable', 'cannot_be_merged': 'conflict'}.get(
                    mr.get('merge_status'), 'checking')
            if status not in ('unchecked', 'checking', 'preparing', 'approvals_syncing') \
                    or time.monotonic() > deadline:
                return status
            time.sleep(5)

    def merge_mr(self, project, iid, sha, message):
        result = self._request('PUT', f'{self._p(project)}/merge_requests/{iid}/merge', body={
            'sha': sha, 'should_remove_source_branch': False, 'merge_commit_message': message,
        })[0]
        deadline = time.monotonic() + 300
        while result.get('state') != 'merged':
            if time.monotonic() > deadline:
                raise ReleaseError(f'{project}: merge request !{iid} did not finish merging')
            time.sleep(5)
            result = self.get(f'{self._p(project)}/merge_requests/{iid}')


# --------------------------------------------------------------------------------------
# Release logic
# --------------------------------------------------------------------------------------

MR_PROBLEMS = {
    'conflict': 'dev and main have merge conflicts',
    'broken_status': 'dev and main have merge conflicts',
    'need_rebase': 'main has commits that are not in dev and only fast-forward merges are allowed',
    'not_approved': 'the merge request needs approval',
    'draft_status': 'the merge request is marked as draft',
    'discussions_not_resolved': 'the merge request has unresolved threads',
    'requested_changes': 'a reviewer requested changes on the merge request',
    'blocked_status': 'the merge request is blocked by another merge request',
    'merge_request_blocked': 'the merge request is blocked by another merge request',
    'not_open': 'the merge request is not open',
}
MR_PIPELINE_PENDING = ('ci_still_running', 'ci_must_pass')
PIPELINE_RUNNING = ('created', 'waiting_for_resource', 'preparing', 'pending', 'running', 'scheduled')
# Kinds whose tag pipeline publishes to GitHub.
MIRRORED_KINDS = ('workspace', 'docs')


@dataclass
class Repo:
    project: str
    kind: str  # component | workspace | docs
    submodule: Optional[str] = None

    @property
    def name(self):
        return self.project.rsplit('/', 1)[-1]


@dataclass
class State:
    repo: Repo
    tag_sha: Optional[str] = None
    dev_sha: Optional[str] = None
    ahead: int = 0
    changes: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)
    problems: list = field(default_factory=list)
    release_notes: str = ''
    mr: Optional[dict] = None

    @property
    def released(self):
        return self.tag_sha is not None


class Releaser:
    def __init__(self, gitlab, version, workspace, own_pipeline=None, author=(None, None),
                 date=None, pipeline_timeout=4 * 3600, poll=30, log=print):
        if not VERSION_RE.match(version or ''):
            raise ReleaseError(f"invalid version '{version}', expected MAJOR.MINOR.PATCH like 0.3.0")
        self.gl = gitlab
        self.version = version
        self.workspace = Repo(workspace, 'workspace')
        self.own_pipeline = own_pipeline
        self.author = author
        self.date = date or datetime.date.today().isoformat()
        self.pipeline_timeout = pipeline_timeout
        self.poll = poll
        self.log = log

    # -- repo discovery ----------------------------------------------------------------

    def submodules(self, repo, ref=DEV):
        """[(path, project)] of the repo's submodules on this GitLab."""
        text = self.gl.read_file(repo.project, '.gitmodules', ref)
        return parse_gitmodules(text, repo.project, self.gl.host) if text else []

    def components(self):
        """The workspace's submodules, each after its own submodules (release order)."""
        if self.gl.read_file(self.workspace.project, '.gitmodules', DEV) is None:
            raise ReleaseError(f'{self.workspace.project}: .gitmodules not found on {DEV}')
        result, seen = [], set()

        def visit(parent, path, project):
            if project in seen:
                return
            seen.add(project)
            repo = Repo(project, 'component', path if parent is self.workspace else None)
            for sub_path, sub_project in self.submodules(repo):
                visit(repo, sub_path, sub_project)
            result.append(repo)
        for path, project in self.submodules(self.workspace):
            visit(self.workspace, path, project)
        return result

    @staticmethod
    def docs():
        return [Repo(project, 'docs') for project in DOCS_REPOS]

    # -- inspection --------------------------------------------------------------------

    def inspect(self, repo: Repo, create_mr: bool) -> State:
        state = State(repo)
        gl, project, version = self.gl, repo.project, self.version
        if repo.kind == 'component' and gl.is_archived(project):
            state.problems.append('the project is archived; remove the submodule or unarchive it')
            return state
        if repo.kind in MIRRORED_KINDS:
            self._check_mirror(state)
        state.tag_sha = gl.tag_sha(project, version)
        main_sha = gl.branch_sha(project, MAIN)
        state.dev_sha = gl.branch_sha(project, DEV)
        if main_sha is None or state.dev_sha is None:
            state.problems.append(f'branch {MAIN} or {DEV} does not exist')
            return state
        if state.released:
            if not gl.is_ancestor(project, state.tag_sha, main_sha):
                state.problems.append(f'tag {version} exists but is not on {MAIN}; fix or delete it by hand')
            else:
                state.notes.append(f'already released ({version} -> {state.tag_sha[:8]})')
            return state

        if repo.kind in ('component', 'workspace'):
            self._check_ros_packages(state)
        elif repo.kind == 'docs':
            self._check_docs(state)

        # The release tags the dev commit, so main must not have changes that dev lacks.
        main_only = gl.changed_on(project, state.dev_sha, main_sha)
        if main_only:
            state.problems.append(f'{MAIN} has changes that are not in {DEV} ({", ".join(main_only[:5])}); '
                                  f'merge {MAIN} into {DEV} first')

        state.ahead = gl.commits_ahead(project, MAIN, state.dev_sha)
        workspace = repo.kind == 'workspace'
        pipeline = gl.latest_pipeline(project, DEV, state.dev_sha, exclude=self.own_pipeline if workspace else None,
                                      skip_releases=workspace)
        if pipeline is None:
            state.problems.append(f'no pipeline ran for {DEV} at {state.dev_sha[:8]}')
        elif pipeline[0] in PIPELINE_RUNNING:
            state.notes.append(f'{DEV} pipeline is {pipeline[0]}, the release waits for it ({pipeline[1]})')
        elif pipeline[0] != 'success':
            state.problems.append(f'latest {DEV} pipeline is {pipeline[0]}: {pipeline[1]}')

        if state.ahead == 0:
            state.notes.append(f'{DEV} has no commits that are not in {MAIN}')
            return state
        state.mr = gl.find_mr(project, DEV, MAIN)
        if state.mr is None and create_mr:
            state.mr = gl.create_mr(project, DEV, MAIN, f'Release {version}',
                                    f'Created by the UFIL release pipeline for {version}.')
            state.notes.append(f'opened merge request {state.mr["web_url"]}')
        if state.mr is not None:
            status = gl.mr_status(project, state.mr['iid'])
            if status != 'mergeable' and status not in MR_PIPELINE_PENDING:
                state.problems.append(self._mr_problem(state, status))
        return state

    def _mr_problem(self, state, status):
        message = MR_PROBLEMS.get(status, f'the merge request is not mergeable ({status})')
        if status in ('conflict', 'broken_status', 'need_rebase'):
            files = self.gl.changed_on(state.repo.project, state.dev_sha, MAIN)
            if files:
                message += f'; {MAIN} changed {", ".join(files[:5])} — merge {MAIN} into {DEV}'
        return f'{message}: {state.mr["web_url"]}'

    def _check_mirror(self, state):
        """The tag pipeline's sync-github-mirror job needs these, else the release stops at GitHub."""
        ci = self.gl.read_file(state.repo.project, '.gitlab-ci.yml', DEV) or ''
        keys = self.gl.variable_keys(state.repo.project)
        for key in MIRROR_VARIABLES:
            if key not in keys and not re.search(rf'^\s+{key}:', ci, re.M):
                state.problems.append(f'CI/CD variable {key} is not set, so the tag pipeline cannot '
                                      'publish to GitHub')

    def _check_ros_packages(self, state):
        gl, project, version = self.gl, state.repo.project, self.version
        entries = gl.tree(project, state.dev_sha, recursive=True)
        paths = {entry['path'] for entry in entries if entry['type'] == 'blob'}
        ignored = {str(PurePosixPath(p).parent) for p in paths
                   if PurePosixPath(p).name in ('COLCON_IGNORE', 'AMENT_IGNORE')}
        notes = []
        changed = [path for path in gl.changed_on(project, MAIN, state.dev_sha)
                   if PurePosixPath(path).name != 'CHANGELOG.rst']
        manifests = sorted(p for p in paths if PurePosixPath(p).name == 'package.xml'
                           and str(PurePosixPath(p).parent) not in ignored)
        changelogs = [(str(PurePosixPath(p).parent), p) for p in manifests]
        if state.repo.kind == 'workspace':
            changelogs.append(('', 'package.xml'))  # root CHANGELOG.rst of the workspace
        for directory, manifest in changelogs:
            # A package at the repo root (ufil-object-tracking) has directory '.'.
            root_package = directory == '.'
            changelog_path = 'CHANGELOG.rst' if not directory or root_package \
                else str(PurePosixPath(directory) / 'CHANGELOG.rst')
            label = directory or 'workspace'
            if directory:
                xml = gl.read_file(project, manifest, state.dev_sha) or ''
                label = package_name(xml) or directory
                if label in EXTERNAL_PACKAGES:
                    continue
                current = package_version(xml)
                if current is None:
                    state.problems.append(f'{label}: package.xml has no <version>')
                elif version_tuple(current) > version_tuple(version):
                    state.problems.append(f'{label}: package.xml version {current} is newer than {version}')
                elif current != version:
                    state.changes[manifest] = set_package_version(xml, version)
            text = gl.read_file(project, changelog_path, state.dev_sha)
            if not directory:
                package_changed = True
            elif root_package:
                package_changed = bool(changed)
            else:
                package_changed = any(path.startswith(directory + '/') for path in changed)
            result, message = check_changelog(text, version, package_changed)
            if result == 'error':
                state.problems.append(f'{label}: {changelog_path}: {message}')
                continue
            if result == 'rename':
                state.changes[changelog_path] = rename_forthcoming(text, version, self.date)
            elif result == 'no_changes':
                state.changes[changelog_path] = add_no_changes(text, version, self.date)
            bullets = section_bullets(text, version)
            notes.append(f'### {label}\n' + '\n'.join(f'* {b}' for b in bullets))
        state.release_notes = '\n\n'.join(notes)

    def _check_docs(self, state):
        conf = self.gl.read_file(state.repo.project, DOCS_CONF, state.dev_sha) or ''
        current = docs_release(conf)
        if current is None or not VERSION_RE.match(current):
            state.problems.append(f"{DOCS_CONF}: has no release = 'MAJOR.MINOR.PATCH'")
        elif version_tuple(current) > version_tuple(self.version):
            state.problems.append(f'{DOCS_CONF}: release {current} is newer than {self.version}')
        elif current != self.version:
            state.changes[DOCS_CONF] = set_docs_release(conf, self.version)
        state.release_notes = (f'Documentation of UFIL {self.version}, published on {DOCS_URL}. '
                               'Build this tag to read the documentation of this release.')

    def preflight(self, repos, create_mrs):
        self.log(f'== Checking {len(repos)} repositories for release {self.version}')
        states = []
        for repo in repos:
            state = self.inspect(repo, create_mrs)
            states.append(state)
            self._report(state)
        problems = [f'{s.repo.name}: {p}' for s in states for p in s.problems]
        if problems:
            raise ReleaseError('Release blocked, nothing was merged or tagged by this step:\n'
                               + '\n'.join(f'  - {p}' for p in problems))
        return states

    def _report(self, state):
        status = 'BLOCKED' if state.problems else ('done' if state.released else 'ready')
        self.log(f'-- {state.repo.name} [{status}]')
        for note in state.notes:
            self.log(f'     {note}')
        for path in sorted(state.changes):
            self.log(f'     will update {path}')
        if not state.released and state.ahead:
            self.log(f'     will merge {state.ahead} commit(s) from {DEV} into {MAIN}')
        for problem in state.problems:
            self.log(f'     PROBLEM: {problem}')

    # -- actions -----------------------------------------------------------------------

    def wait_for_pipeline(self, repo, sha, ref=DEV):
        exclude = self.own_pipeline if repo.kind == 'workspace' else None
        deadline = time.monotonic() + self.pipeline_timeout
        missing_since = time.monotonic()
        announced = None
        while True:
            pipeline = self.gl.latest_pipeline(repo.project, ref, sha, exclude=exclude,
                                               skip_releases=repo.kind == 'workspace')
            if pipeline is None:
                if time.monotonic() - missing_since > 600:
                    raise ReleaseError(f'{repo.name}: no pipeline started for {ref} at {sha[:8]}')
            else:
                status, url = pipeline
                if status == 'success':
                    return
                if status not in PIPELINE_RUNNING:
                    raise ReleaseError(f'{repo.name}: {ref} pipeline {status}: {url}')
                if announced != url:
                    self.log(f'   waiting for {repo.name} {ref} pipeline: {url}')
                    announced = url
            if time.monotonic() > deadline:
                raise ReleaseError(f'{repo.name}: timed out waiting for the {ref} pipeline')
            time.sleep(self.poll)

    def _release_notes_of_tag(self, repo):
        tagged = State(repo, dev_sha=self.gl.tag_sha(repo.project, self.version))
        if repo.kind == 'docs':
            self._check_docs(tagged)
        else:
            self._check_ros_packages(tagged)
        return tagged.release_notes

    def merge_and_tag(self, state, prepare=True):
        """Finalise dev, merge it into main and tag the merged dev commit. Returns the tag commit."""
        repo, gl, version = state.repo, self.gl, self.version
        if state.released:
            if not gl.release_exists(repo.project, version):
                gl.create_release(repo.project, version, f'Version {version}', self._release_notes_of_tag(repo))
                self.log(f'   created the missing GitLab release for {repo.name} {version}')
            return state.tag_sha
        self.log(f'== Releasing {repo.name}')
        sha = state.dev_sha
        if prepare and state.changes:
            sha = gl.commit_files(repo.project, DEV, f'Prepare release {version}',
                                  state.changes, self.author)
            self.log(f'   committed changelog and version updates to {DEV} ({sha[:8]})')
        if gl.commits_ahead(repo.project, MAIN, sha):
            self.wait_for_pipeline(repo, sha)
            mr = gl.find_mr(repo.project, DEV, MAIN) or gl.create_mr(
                repo.project, DEV, MAIN, f'Release {version}',
                f'Created by the UFIL release pipeline for {version}.')
            deadline = time.monotonic() + 600
            while True:
                status = gl.mr_status(repo.project, mr['iid'])
                if status == 'mergeable':
                    break
                if status not in MR_PIPELINE_PENDING or time.monotonic() > deadline:
                    state.mr, state.dev_sha = mr, sha
                    raise ReleaseError(f'{repo.name}: {self._mr_problem(state, status)}')
                time.sleep(self.poll)
            gl.merge_mr(repo.project, mr['iid'], sha, f"Merge branch '{DEV}' into '{MAIN}' for release {version}")
            self.log(f'   merged {mr["web_url"]}')
        main_sha = gl.branch_sha(repo.project, MAIN)
        if not gl.is_ancestor(repo.project, sha, main_sha):
            raise ReleaseError(f'{repo.name}: {MAIN} does not contain {DEV} ({sha[:8]}) after merging')
        # The tag goes on the merged dev commit, so it is reachable from dev and main.
        if not gl.same_files(repo.project, sha, main_sha):
            raise ReleaseError(f'{repo.name}: {MAIN} ({main_sha[:8]}) has other files than the {DEV} commit '
                               f'{sha[:8]}; merge {MAIN} into {DEV} and run the release again')
        gl.create_tag(repo.project, version, sha, f'Release {version}')
        gl.create_release(repo.project, version, f'Version {version}', state.release_notes)
        self.log(f'   tagged {DEV} commit {sha[:8]} as {version} ({MAIN} is {main_sha[:8]})')
        state.tag_sha = sha
        return sha

    def release_with_submodules(self, state, tags):
        """Point the submodules to the released repos (tags: project -> tag commit), then merge
        and tag. Used for the workspace and for components with submodules (ufil-core)."""
        repo, gl = state.repo, self.gl
        if state.released:
            return self.merge_and_tag(state)
        modules = self.submodules(repo)
        if not modules:
            return self.merge_and_tag(state)
        self.log(f'== Releasing {repo.name}')
        current = {}
        for parent in sorted({str(PurePosixPath(path).parent) for path, _ in modules}):
            current.update({entry['path']: entry['id']
                            for entry in gl.tree(repo.project, DEV, '' if parent == '.' else parent)
                            if entry['type'] == 'commit'})
        submodules = {}
        for path, project in modules:
            if project not in tags:
                raise ReleaseError(f'{repo.name}: submodule {path} ({project}) was not released')
            sha = tags[project]
            if current.get(path) != sha:
                submodules[path] = sha
                self.log(f'   {path}: {str(current.get(path))[:8]} -> {sha[:8]} ({self.version})')
        sha = state.dev_sha
        if state.changes or submodules:
            sha = gl.commit_workspace(repo.project, DEV, f'Release {self.version}', state.changes,
                                      submodules, self.author) or sha
            self.log(f'   committed submodule, changelog and version updates to {DEV} ({sha[:8]})')
        state.dev_sha = sha
        return self.merge_and_tag(state, prepare=False)

    def wait_for_github(self, states):
        """The tag pipelines of the workspace and the documentation publish to GitHub."""
        for state in states:
            if state.repo.kind in MIRRORED_KINDS:
                self.wait_for_pipeline(state.repo, state.tag_sha, ref=self.version)
                self.log(f'   {state.repo.name} {self.version} is published on GitHub')

    # -- commands ----------------------------------------------------------------------

    def all_repos(self):
        return self.components() + [self.workspace] + self.docs()

    def check(self):
        self.preflight(self.all_repos(), create_mrs=False)
        self.log(f'== All checks passed for {self.version}. Nothing was changed.')

    def release(self):
        repos = self.all_repos()
        states = self.preflight(repos, create_mrs=True)
        tags = {}
        for state in states:
            if state.repo.kind == 'docs':
                self.merge_and_tag(state)
            else:
                tags[state.repo.project] = self.release_with_submodules(state, tags)
        self.log(f'== All repositories tagged {self.version}; waiting for the GitHub publication.')
        self.wait_for_github(states)
        self.log(f'== Released UFIL {self.version}.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=['check', 'release'])
    parser.add_argument('--version', help='defaults to RELEASE_VERSION')
    args = parser.parse_args(argv)
    env = os.environ
    version = (args.version or env.get('RELEASE_VERSION') or '').strip()
    try:
        token = env.get('RELEASE_TOKEN')
        if not token:
            raise ReleaseError('RELEASE_TOKEN is not set (group access token with api scope)')
        releaser = Releaser(
            GitLab(env.get('CI_SERVER_URL', DEFAULT_SERVER), token), version,
            env.get('CI_PROJECT_PATH', DEFAULT_WORKSPACE), own_pipeline=env.get('CI_PIPELINE_ID'),
            author=(env.get('GITLAB_USER_NAME'), env.get('GITLAB_USER_EMAIL')),
            pipeline_timeout=int(env.get('PIPELINE_TIMEOUT', 4 * 3600)))
        {'check': releaser.check, 'release': releaser.release}[args.command]()
        return 0
    except ReleaseError as error:
        print(f'\nERROR: {error}', file=sys.stderr)
        print('Fix the problem and run the job again; finished steps are skipped.', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
