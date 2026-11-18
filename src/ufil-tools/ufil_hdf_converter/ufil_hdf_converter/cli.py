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

"""Command line interface ``ufil_hdf_converter``."""

import argparse
import datetime
import logging
import os
import sys
from typing import List, Optional

from .hdf_writer import FileMetadata

_LOGGER = logging.getLogger('ufil_hdf_converter')


def output_filename(bag: str, output: Optional[str], key: Optional[str],
                    today: Optional[datetime.date] = None) -> str:
    """
    Choose the HDF5 file name.

    An explicit ``output`` wins. A ``key`` yields the SFB name ``B03_SSC_<yymmdd>_<key>.hdf5``
    in the working directory. Otherwise the SFB name with key ``NOKEY`` is placed next to the
    bag data.
    """
    if output is not None:
        return output
    date_string = (today or datetime.date.today()).strftime('%y%m%d')
    if key is not None:
        return f'B03_SSC_{date_string}_{key}.hdf5'
    directory = bag if os.path.isdir(bag) else os.path.dirname(bag)
    return os.path.join(directory, f'B03_SSC_{date_string}_NOKEY.hdf5')


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='ufil_hdf_converter',
        description='Convert a rosbag2 bag into an HDF5 file following the SFB data handbook.')
    parser.add_argument('bag', help='rosbag2 bag directory or file')
    naming = parser.add_mutually_exclusive_group()
    naming.add_argument('-o', '--out', help='output HDF5 file')
    naming.add_argument('-k', '--key', help='key of the SFB output file name')
    parser.add_argument('-t', '--topic', nargs='+', dest='topics', metavar='TOPIC',
                        help='topics to convert (default: all)')
    parser.add_argument('-f', '--force', action='store_true',
                        help='overwrite an existing output file')
    parser.add_argument('-d', '--description', default=FileMetadata.description,
                        help='description stored in the file attributes')
    parser.add_argument('-v', '--verbose', action='store_true', help='print debug output')
    return parser


def _print_progress(processed: int, total: int) -> None:
    if total and (processed == total or processed % max(total // 100, 1) == 0):
        sys.stderr.write(f'\rConverting: {100 * processed // total:3d} %')
        if processed == total:
            sys.stderr.write('\n')


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format='[%(levelname)s] %(message)s')

    output = output_filename(args.bag, args.out, args.key)
    if os.path.exists(output) and not args.force:
        _LOGGER.error('%s already exists. Remove it or pass --force to overwrite it.', output)
        return 2

    # Imported here so that --help works without a sourced ROS 2 environment.
    from .bag_converter import convert_bag

    _LOGGER.info('Converting %s to %s.', args.bag, output)
    try:
        stats = convert_bag(args.bag, output, args.topics,
                            FileMetadata(description=args.description),
                            _print_progress if sys.stderr.isatty() else None)
    except (OSError, RuntimeError, ValueError) as error:
        _LOGGER.error('%s', error)
        return 1

    for (topic, topic_type), count in sorted(stats.unsupported_topics.items()):
        _LOGGER.warning("Skipped %d messages on '%s': type '%s' is not supported.",
                        count, topic, topic_type)
    _LOGGER.info('Wrote %d messages, skipped %d.', stats.written, stats.skipped)
    return 0


if __name__ == '__main__':
    sys.exit(main())
