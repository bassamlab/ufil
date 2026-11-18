#!/usr/bin/env python3
# Copyright 2026 UniBw M
"""Tests for release.py. Run with: python3 -m unittest discover -s ci"""

import itertools
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import release  # noqa: E402
from release import (ReleaseError, Releaser, check_changelog, parse_gitmodules,  # noqa: E402
                     rename_forthcoming, set_package_version, url_host)

G = 'cas/project/simon/software/ufil/ufil-components'
WS = 'cas/project/simon/software/ufil/ufil-workspaces/ufil-workspace'
DOCS = 'cas/project/simon/software/documentation'
CORE, OSN, TRACKING = f'{G}/ufil-core', f'{G}/ufil-osn', f'{G}/ufil-object-tracking'
HOST = 'git.unibw.de'


def changelog(name, top, bullets=('Something changed',)):
    lines = ['^' * 20, f'Changelog for package {name}', '^' * 20, '', top, '-' * len(top)]
    lines += [f'* {b}' for b in bullets]
    lines += ['', '0.2.0 (2026-10-30)', '------------------', '* Old change', '']
    return '\n'.join(lines)


def package_xml(name, version):
    return f'<package format="3">\n  <name>{name}</name>\n  <version>{version}</version>\n</package>\n'


def gitmodule(path, url):
    return f'[submodule "{path}"]\n\tpath = {path}\n\turl = {url}\n'


def ssh(project):
    return f'ssh://git@{HOST}:2222/{project}.git'


class FakeProject:
    counter = itertools.count()

    def __init__(self, files, subs=None):
        self.commits = {}
        root = self._commit([], files, subs or {})
        self.branches = {'main': root, 'dev': root}
        self.tags, self.releases, self.mrs = {}, {}, []
        self.pipeline = {}  # sha -> status, default success
        self.tag_pipeline = {}  # tag -> status, default success
        self.mr_status = None  # forced detailed merge status
        self.archived = False
        self.variables = set()

    def _commit(self, parents, files, subs):
        sha = f'{next(self.counter):040x}'
        self.commits[sha] = {'parents': parents, 'files': dict(files), 'subs': dict(subs), 'n': int(sha, 16)}
        return sha

    def commit(self, branch, files=None, subs=None):
        head = self.commits[self.branches[branch]]
        sha = self._commit([self.branches[branch]], {**head['files'], **(files or {})},
                           {**head['subs'], **(subs or {})})
        self.branches[branch] = sha
        return sha

    def resolve(self, ref):
        return self.branches.get(ref) or self.tags.get(ref) or ref

    def ancestors(self, sha):
        seen, todo = set(), [sha]
        while todo:
            current = todo.pop()
            if current not in seen:
                seen.add(current)
                todo.extend(self.commits[current]['parents'])
        return seen


class FakeGitLab:
    """In-memory stand-in for release.GitLab."""

    host = HOST

    def __init__(self, projects):
        self.p = projects
        self.calls = []

    def is_archived(self, project):
        return self.p[project].archived

    def variable_keys(self, project):
        return self.p[project].variables

    def branch_sha(self, project, branch):
        return self.p[project].branches.get(branch)

    def tag_sha(self, project, tag):
        return self.p[project].tags.get(tag)

    def is_ancestor(self, project, ancestor, ref):
        repo = self.p[project]
        return repo.resolve(ancestor) in repo.ancestors(repo.resolve(ref))

    def commits_ahead(self, project, base, head):
        repo = self.p[project]
        return len(repo.ancestors(repo.resolve(head)) - repo.ancestors(repo.resolve(base)))

    def same_files(self, project, a, b):
        repo = self.p[project]
        a, b = repo.commits[repo.resolve(a)], repo.commits[repo.resolve(b)]
        return a['files'] == b['files'] and a['subs'] == b['subs']

    def changed_on(self, project, base, head):
        repo = self.p[project]
        head = repo.resolve(head)
        common = repo.ancestors(repo.resolve(base)) & repo.ancestors(head)
        merge_base = max(common, key=lambda sha: repo.commits[sha]['n'])
        old, new = repo.commits[merge_base]['files'], repo.commits[head]['files']
        return sorted(p for p in set(old) | set(new) if old.get(p) != new.get(p))

    def read_file(self, project, path, ref):
        repo = self.p[project]
        return repo.commits[repo.resolve(ref)]['files'].get(path)

    def tree(self, project, ref, path='', recursive=False):
        repo = self.p[project]
        commit = repo.commits[repo.resolve(ref)]
        entries = [{'path': p, 'type': 'blob', 'id': 'x'} for p in commit['files']]
        entries += [{'path': p, 'type': 'commit', 'id': s} for p, s in commit['subs'].items()]
        prefix = path + '/' if path else ''
        result = {}
        for entry in entries:
            if not entry['path'].startswith(prefix):
                continue
            rest = entry['path'][len(prefix):]
            if recursive or '/' not in rest:
                result[entry['path']] = entry
            else:
                directory = prefix + rest.split('/')[0]
                result[directory] = {'path': directory, 'type': 'tree', 'id': 'x'}
        return list(result.values())

    def commit_files(self, project, branch, message, files, author):
        self.calls.append(('commit', project))
        return self.p[project].commit(branch, files)

    def commit_workspace(self, project, branch, message, files, submodules, author):
        self.calls.append(('commit', project))
        return self.p[project].commit(branch, files, submodules)

    def create_tag(self, project, tag, ref, message):
        repo = self.p[project]
        assert tag not in repo.tags, 'tags must never be overwritten'
        repo.tags[tag] = repo.resolve(ref)
        self.calls.append(('tag', project))

    def release_exists(self, project, tag):
        return tag in self.p[project].releases

    def create_release(self, project, tag, name, description):
        self.p[project].releases[tag] = description

    def latest_pipeline(self, project, ref, sha, exclude=None, skip_releases=False):
        repo = self.p[project]
        status = repo.tag_pipeline.get(ref, 'success') if ref in repo.tags else repo.pipeline.get(sha, 'success')
        return None if status is None else (status, f'https://ci/{project}/{ref}/{sha[:6]}')

    def find_mr(self, project, source, target):
        open_mrs = [mr for mr in self.p[project].mrs if mr['state'] == 'opened']
        return open_mrs[0] if open_mrs else None

    def create_mr(self, project, source, target, title, description):
        repo = self.p[project]
        mr = {'iid': len(repo.mrs) + 1, 'state': 'opened', 'web_url': f'https://mr/{project}'}
        repo.mrs.append(mr)
        self.calls.append(('mr', project))
        return mr

    def mr_status(self, project, iid, timeout=0):
        return self.p[project].mr_status or 'mergeable'

    def merge_mr(self, project, iid, sha, message):
        repo = self.p[project]
        assert repo.branches['dev'] == sha
        dev = repo.commits[sha]
        repo.branches['main'] = repo._commit([repo.branches['main'], sha], dev['files'], dev['subs'])
        repo.mrs[iid - 1]['state'] = 'merged'
        self.calls.append(('merge', project))


def ros_component(pkg, prefix=''):
    """A component with one package; prefix '' puts the package at the repo root."""
    project = FakeProject({
        f'{prefix}package.xml': package_xml(pkg, '0.2.0'),
        f'{prefix}CHANGELOG.rst': changelog(pkg, '0.2.0 (2026-10-30)'),
        f'{prefix}src/node.cpp': 'v1',
    })
    project.commit('dev', {f'{prefix}CHANGELOG.rst': changelog(pkg, 'Forthcoming'),
                           f'{prefix}src/node.cpp': 'v2'})
    return project


def make_world():
    projects = {TRACKING: ros_component('ufil_object_tracking')}
    projects[CORE] = ros_component('ufil_core', 'ufil_core/')
    projects[CORE].commit('dev', {'.gitmodules': gitmodule('ufil-object-tracking', ssh(TRACKING))},
                          {'ufil-object-tracking': projects[TRACKING].branches['main']})
    projects[OSN] = ros_component('ufil_osn', 'ufil_osn/')
    # External submodules (GitHub) are not part of the release.
    projects[OSN].commit('dev', {'.gitmodules': gitmodule(
        'carla/ros-carla-msgs', 'https://github.com/carla-simulator/ros-carla-msgs.git')},
        {'carla/ros-carla-msgs': 'f' * 40})
    projects[WS] = FakeProject({
        '.gitmodules': gitmodule('src/ufil-osn', ssh(OSN)) + gitmodule('src/ufil-core', ssh(CORE)),
        'CHANGELOG.rst': changelog('UFIL', '0.2.0 (2026-10-30)'),
        '.gitlab-ci.yml': 'variables:\n  GITHUB_MIRROR_BRANCH: main\n',
    }, {'src/ufil-osn': projects[OSN].branches['main'], 'src/ufil-core': projects[CORE].branches['main']})
    projects[WS].commit('dev', {'CHANGELOG.rst': changelog('UFIL', 'Forthcoming', ['Release automation'])})
    projects[WS].variables = {'GITHUB_MIRROR_DEPLOY_KEY', 'GITHUB_MIRROR_URL'}
    projects[DOCS] = FakeProject({
        'src/conf.py': "project = 'UFIL'\nrelease = '0.2.0'\n",
        '.gitlab-ci.yml': 'variables:\n  GITHUB_MIRROR_URL: git@github.com:bassamlab/ufil-docs.git\n',
    })
    projects[DOCS].commit('dev', {'src/index.rst': 'new page'})
    projects[DOCS].variables = {'GITHUB_MIRROR_DEPLOY_KEY'}
    return projects


class ReleaseFlowTest(unittest.TestCase):

    def setUp(self):
        self.projects = make_world()
        self.gl = FakeGitLab(self.projects)
        self.log = []

    def releaser(self):
        return Releaser(self.gl, '0.3.0', WS, own_pipeline='1', date='2026-11-18',
                        poll=0, log=self.log.append)

    def tagged(self, project, path=None):
        repo = self.projects[project]
        commit = repo.commits[repo.tags['0.3.0']]
        return commit if path is None else commit['files'].get(path, commit['subs'].get(path))

    def test_happy_path(self):
        self.releaser().release()
        tag_order = [project for kind, project in self.gl.calls if kind == 'tag']
        # Nested submodules first, the workspace after its components, the documentation last.
        self.assertEqual(tag_order, [OSN, TRACKING, CORE, WS, DOCS])
        for project in tag_order:
            repo = self.projects[project]
            # The merged dev commit is tagged: on dev and main, and main has the same files.
            self.assertEqual(repo.tags['0.3.0'], repo.branches['dev'], project)
            self.assertTrue(self.gl.is_ancestor(project, '0.3.0', 'main'), project)
            self.assertTrue(self.gl.same_files(project, '0.3.0', 'main'), project)
            self.assertIn('0.3.0', repo.releases, project)
        # A package at the repo root (ufil-object-tracking).
        self.assertIn('<version>0.3.0</version>', self.tagged(TRACKING, 'package.xml'))
        self.assertIn('\n0.3.0 (2026-11-18)\n------------------\n', self.tagged(TRACKING, 'CHANGELOG.rst'))
        self.assertIn('### ufil_object_tracking\n* Something changed', self.projects[TRACKING].releases['0.3.0'])
        # ufil-core points to the released ufil-object-tracking.
        self.assertEqual(self.tagged(CORE, 'ufil-object-tracking'), self.projects[TRACKING].tags['0.3.0'])
        self.assertIn('<version>0.3.0</version>', self.tagged(CORE, 'ufil_core/package.xml'))
        # The external submodule is left as it is.
        self.assertEqual(self.tagged(OSN, 'carla/ros-carla-msgs'), 'f' * 40)
        self.assertEqual(self.tagged(WS, 'src/ufil-core'), self.projects[CORE].tags['0.3.0'])
        self.assertEqual(self.tagged(WS, 'src/ufil-osn'), self.projects[OSN].tags['0.3.0'])
        self.assertEqual(self.tagged(WS, 'CHANGELOG.rst').splitlines()[4], '0.3.0 (2026-11-18)')
        self.assertIn('Release automation', self.projects[WS].releases['0.3.0'])
        self.assertEqual(self.tagged(DOCS, 'src/conf.py'), "project = 'UFIL'\nrelease = '0.3.0'\n")
        self.assertIn('Documentation of UFIL 0.3.0', self.projects[DOCS].releases['0.3.0'])
        self.assertIn('ufil-workspace 0.3.0 is published on GitHub', '\n'.join(self.log))
        self.assertIn('documentation 0.3.0 is published on GitHub', '\n'.join(self.log))

    def test_rerun_after_success_changes_nothing(self):
        self.releaser().release()
        calls = len(self.gl.calls)
        self.releaser().release()
        self.assertEqual(len(self.gl.calls), calls)

    def test_check_changes_nothing(self):
        self.releaser().check()
        self.assertEqual(self.gl.calls, [])

    def test_conflict_blocks_before_any_merge(self):
        self.projects[CORE].mr_status = 'conflict'
        with self.assertRaises(ReleaseError) as ctx:
            self.releaser().release()
        self.assertIn('ufil-core: dev and main have merge conflicts', str(ctx.exception))
        self.assertFalse([c for c in self.gl.calls if c[0] in ('merge', 'tag', 'commit')])

    def test_check_reports_all_problems(self):
        self.projects[OSN].commit('dev', {'ufil_osn/CHANGELOG.rst': changelog('ufil_osn', '0.2.0 (2026-10-30)')})
        self.projects[CORE].pipeline[self.projects[CORE].branches['dev']] = 'failed'
        self.projects[DOCS].variables = set()
        with self.assertRaises(ReleaseError) as ctx:
            self.releaser().check()
        message = str(ctx.exception)
        self.assertIn('ufil_osn: ufil_osn/CHANGELOG.rst: changed since the last release but has no entry for '
                      '0.3.0', message)
        self.assertIn('ufil-core: latest dev pipeline is failed', message)
        self.assertIn('documentation: CI/CD variable GITHUB_MIRROR_DEPLOY_KEY is not set', message)
        # Defined in the documentation's .gitlab-ci.yml.
        self.assertNotIn('documentation: CI/CD variable GITHUB_MIRROR_URL', message)

    def test_archived_component_blocks_the_release(self):
        self.projects[TRACKING].archived = True
        with self.assertRaises(ReleaseError) as ctx:
            self.releaser().check()
        self.assertIn('ufil-object-tracking: the project is archived', str(ctx.exception))

    def test_failure_midway_resumes(self):
        # The ufil-core pipeline fails after the release commit; the components before it are released.
        original = self.gl.commit_workspace

        def failing_commit(project, *args):
            sha = original(project, *args)
            if project == CORE:
                self.projects[project].pipeline[sha] = 'failed'
            return sha
        self.gl.commit_workspace = failing_commit
        with self.assertRaises(ReleaseError) as ctx:
            self.releaser().release()
        self.assertIn('ufil-core: dev pipeline failed', str(ctx.exception))
        self.assertIn('0.3.0', self.projects[TRACKING].tags)
        self.assertNotIn('0.3.0', self.projects[CORE].tags)
        self.assertNotIn('0.3.0', self.projects[WS].tags)

        # Someone fixes dev; the rerun skips the released repos and finishes the release.
        self.gl.commit_workspace = original
        self.projects[CORE].commit('dev', {'fix': 'fixed'})
        tracking_tag = self.projects[TRACKING].tags['0.3.0']
        self.releaser().release()
        self.assertEqual(self.projects[TRACKING].tags['0.3.0'], tracking_tag)
        self.assertEqual(self.tagged(CORE, 'ufil-object-tracking'), tracking_tag)
        self.assertIn('0.3.0', self.projects[DOCS].tags)

    def test_github_publication_failure_is_reported_and_resumes(self):
        self.projects[WS].tag_pipeline['0.3.0'] = 'failed'
        with self.assertRaises(ReleaseError) as ctx:
            self.releaser().release()
        self.assertIn('ufil-workspace: 0.3.0 pipeline failed', str(ctx.exception))
        self.assertIn('0.3.0', self.projects[DOCS].tags, 'everything is tagged before publishing')
        self.projects[WS].tag_pipeline['0.3.0'] = 'success'  # job retried after fixing the deploy key
        calls = len(self.gl.calls)
        self.releaser().release()
        self.assertEqual(len(self.gl.calls), calls)

    def test_unchanged_package_gets_no_changes_entry(self):
        osn = self.projects[OSN]
        osn.branches['dev'] = osn.branches['main']
        osn.commit('dev', {'README.md': 'docs only'})
        self.releaser().release()
        text = self.tagged(OSN, 'ufil_osn/CHANGELOG.rst')
        self.assertIn('\n0.3.0 (2026-11-18)\n------------------\n* No changes\n\n0.2.0 (2026-10-30)\n', text)
        self.assertIn('No changes', osn.releases['0.3.0'])

    def test_external_package_keeps_its_version(self):
        driver = {'uofa/sbg_driver/package.xml': package_xml('sbg_driver', '3.1.1'),
                  'uofa/sbg_driver/CHANGELOG.rst': changelog('sbg_driver', '3.1.1 (2026-10-30)')}
        self.projects[OSN].commit('dev', driver)
        self.releaser().release()
        for path, text in driver.items():
            self.assertEqual(self.tagged(OSN, path), text)
        self.assertNotIn('sbg_driver', self.projects[OSN].releases['0.3.0'])

    def test_main_changes_missing_in_dev_block_the_release(self):
        self.projects[DOCS].commit('main', {'hotfix': 'only on main'})
        with self.assertRaises(ReleaseError) as ctx:
            self.releaser().release()
        self.assertIn('documentation: main has changes that are not in dev (hotfix)', str(ctx.exception))
        self.assertFalse([c for c in self.gl.calls if c[0] in ('merge', 'tag', 'commit')])

    def test_main_with_other_files_after_merge_is_not_tagged(self):
        original = self.gl.merge_mr

        def merge_with_extra_file(project, iid, sha, message):
            original(project, iid, sha, message)
            if project == OSN:
                self.projects[project].commit('main', {'unexpected': 'x'})
        self.gl.merge_mr = merge_with_extra_file
        with self.assertRaisesRegex(ReleaseError, 'ufil-osn: main .* has other files than the dev commit'):
            self.releaser().release()
        self.assertNotIn('0.3.0', self.projects[OSN].tags)

    def test_tag_not_on_main_is_a_problem(self):
        osn = self.projects[OSN]
        osn.tags['0.3.0'] = osn.branches['dev']
        with self.assertRaises(ReleaseError) as ctx:
            self.releaser().check()
        self.assertIn('tag 0.3.0 exists but is not on main', str(ctx.exception))

    def test_missing_docs_release_is_created(self):
        docs = self.projects[DOCS]
        docs.commit('dev', {'src/conf.py': "release = '0.3.0'\n"})
        docs.branches['main'] = docs.branches['dev']
        docs.tags['0.3.0'] = docs.branches['dev']
        self.releaser().release()
        self.assertIn('Documentation of UFIL 0.3.0', docs.releases['0.3.0'])

    def test_docs_release_newer_than_version_is_a_problem(self):
        self.projects[DOCS].commit('dev', {'src/conf.py': "release = '0.4.0'\n"})
        with self.assertRaises(ReleaseError) as ctx:
            self.releaser().check()
        self.assertIn('documentation: src/conf.py: release 0.4.0 is newer than 0.3.0', str(ctx.exception))

    def test_invalid_version(self):
        with self.assertRaises(ReleaseError):
            Releaser(self.gl, 'v0.3', WS)


class ChangelogTest(unittest.TestCase):

    def test_check(self):
        self.assertEqual(check_changelog(changelog('p', 'Forthcoming'), '0.3.0')[0], 'rename')
        self.assertEqual(check_changelog(changelog('p', '0.3.0 (2026-11-18)'), '0.3.0')[0], 'ok')
        self.assertEqual(check_changelog(changelog('p', 'Forthcoming', []), '0.3.0')[0], 'error')
        self.assertEqual(check_changelog(changelog('p', '0.2.0 (2026-10-30)'), '0.3.0')[0], 'error')
        self.assertEqual(check_changelog(changelog('p', '0.4.0 (2026-12-02)'), '0.3.0')[0], 'error')
        self.assertEqual(check_changelog(None, '0.3.0')[0], 'error')
        self.assertEqual(check_changelog(changelog('p', '0.2.0 (2026-10-30)'), '0.3.0', changed=False)[0],
                         'no_changes')
        # Forthcoming on top of an existing entry for the release version would duplicate it.
        self.assertEqual(check_changelog(changelog('p', 'Forthcoming'), '0.2.0')[0], 'error')

    def test_rename(self):
        text = rename_forthcoming(changelog('p', 'Forthcoming'), '0.3.0', '2026-11-18')
        self.assertIn('\n0.3.0 (2026-11-18)\n------------------\n* Something changed\n', text)
        self.assertEqual(check_changelog(text, '0.3.0')[0], 'ok')

    def test_package_xml(self):
        xml = package_xml('p', '0.2.0').replace('</package>', '  <depend version_gte="1.0.0">x</depend>\n</package>')
        self.assertIn('<version>0.3.0</version>', set_package_version(xml, '0.3.0'))
        self.assertIn('version_gte="1.0.0"', set_package_version(xml, '0.3.0'))

    def test_gitmodules(self):
        text = ('[submodule "src/a"]\n\tpath = src/a\n\turl = git@git.unibw.de:G/c/a.git\n'
                '[submodule "src/b"]\n\tpath = src/b\n\turl = ssh://git@git.unibw.de:2222/G/c/b.git\n'
                '[submodule "src/c"]\n\tpath = src/c\n\turl = ../../c/c.git\n'
                '[submodule "src/d"]\n\tpath = src/d\n\turl = https://github.com/o/d.git\n')
        internal = [('src/a', 'G/c/a'), ('src/b', 'G/c/b'), ('src/c', 'G/c/c')]
        self.assertEqual(parse_gitmodules(text, 'G/w/ws'), internal + [('src/d', 'o/d')])
        self.assertEqual(parse_gitmodules(text, 'G/w/ws', 'git.unibw.de'), internal)

    def test_url_host(self):
        self.assertEqual(url_host('git@git.unibw.de:G/a.git'), 'git.unibw.de')
        self.assertEqual(url_host('ssh://git@git.unibw.de:2222/G/a.git'), 'git.unibw.de')
        self.assertEqual(url_host('https://github.com/o/d.git'), 'github.com')
        self.assertIsNone(url_host('../a.git'))


class GitLabClientTest(unittest.TestCase):

    def test_tree_passes_path_as_query_parameter(self):
        gl = release.GitLab('https://gitlab.example', 'token')
        requests = []

        def fake_request(method, endpoint, params=None, body=None, raw=False):
            requests.append((method, endpoint, dict(params)))
            return [{'path': 'src/a', 'type': 'tree', 'id': 'x'}], None
        gl._request = fake_request
        entries = gl.tree('G/ws', 'dev', 'src')
        self.assertEqual(entries[0]['path'], 'src/a')
        self.assertEqual(requests[0][1], '/projects/G%2Fws/repository/tree')
        self.assertEqual(requests[0][2]['path'], 'src')
        self.assertEqual(gl.host, 'gitlab.example')

    def test_latest_pipeline_skips_release_pipelines(self):
        gl = release.GitLab('https://gitlab.example', 'token')
        responses = {
            '/projects/G%2Fws/pipelines': [{'id': 3, 'status': 'failed', 'web_url': 'dry-run'},
                                           {'id': 2, 'status': 'success', 'web_url': 'push'}],
            '/projects/G%2Fws/pipelines/3/variables': [{'key': 'RELEASE_VERSION', 'value': '0.3.0'}],
            '/projects/G%2Fws/pipelines/2/variables': [],
        }
        gl._request = lambda method, endpoint, params=None, body=None, raw=False: (responses[endpoint], None)
        self.assertEqual(gl.latest_pipeline('G/ws', 'dev', 'abc'), ('failed', 'dry-run'))
        self.assertEqual(gl.latest_pipeline('G/ws', 'dev', 'abc', skip_releases=True), ('success', 'push'))


if __name__ == '__main__':
    release.time.sleep = lambda *_: None
    unittest.main()
