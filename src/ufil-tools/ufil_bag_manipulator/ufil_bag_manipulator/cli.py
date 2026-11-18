# Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Command line interface ``ufil_bag_manipulator``."""

import argparse
import logging
import sys
from typing import List, Optional

from .bag_io import DEFAULT_STORAGE_ID, merge_bags, trim_bag

_LOGGER = logging.getLogger('ufil_bag_manipulator')


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='ufil_bag_manipulator',
        description='Merge rosbag2 bags or trim the start of a bag and shift its timestamps.')
    parser.add_argument('-v', '--verbose', action='store_true', help='print debug output')
    commands = parser.add_subparsers(dest='command', required=True)

    storage_help = f'storage plugin of the output bag (default: {DEFAULT_STORAGE_ID})'

    merge = commands.add_parser('merge', help='merge bags into one bag ordered by receive time')
    merge.add_argument('inputs', nargs='+', metavar='INPUT', help='input bag directories')
    merge.add_argument('-o', '--output', required=True, help='output bag directory')
    merge.add_argument('--storage', default=DEFAULT_STORAGE_ID, help=storage_help)

    trim = commands.add_parser(
        'trim', help='drop the start of a bag and shift the remaining timestamps to zero')
    trim.add_argument('input', help='input bag directory')
    trim.add_argument('-o', '--output', required=True, help='output bag directory')
    trim.add_argument('--trim', type=float, default=0.0, metavar='SECONDS',
                      help='duration to drop after the first message (default: 0)')
    trim.add_argument('--keep-timestamps', action='store_true',
                      help='keep the original timestamps instead of shifting the cutoff to 0')
    trim.add_argument('--storage', default=DEFAULT_STORAGE_ID, help=storage_help)
    return parser


def _merge(args: argparse.Namespace) -> None:
    if len(args.inputs) < 2:
        raise ValueError('merge needs at least two input bags.')
    _LOGGER.info('Merging %s into %s.', ', '.join(args.inputs), args.output)
    stats = merge_bags(args.inputs, args.output, args.storage)
    for uri, count in zip(args.inputs, stats.messages_per_input):
        _LOGGER.debug('%d messages from %s.', count, uri)
    _LOGGER.info('Wrote %d messages to %s.', stats.total, args.output)


def _trim(args: argparse.Namespace) -> None:
    _LOGGER.info('Trimming the first %.3f s of %s into %s.', args.trim, args.input, args.output)
    stats = trim_bag(args.input, args.output, args.trim, not args.keep_timestamps,
                     args.storage)
    if stats.start_timestamp is None:
        _LOGGER.warning('%s contains no messages; wrote an empty bag.', args.input)
        return
    _LOGGER.info('Dropped %d messages before %d ns, wrote %d messages to %s.',
                 stats.dropped, stats.cutoff_timestamp, stats.kept, args.output)


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format='[%(levelname)s] %(message)s')
    try:
        {'merge': _merge, 'trim': _trim}[args.command](args)
    except (OSError, RuntimeError, ValueError) as error:
        _LOGGER.error('%s', error)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
