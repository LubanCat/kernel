#!/usr/bin/env python

# The format of the kernel configs in the framework compatibility matrix
# has a couple properties that would make it confusing or cumbersome to
# maintain by hand:
#
#  - Conditions apply to all configs within the same <kernel> section.
#    The <kernel> tag also specifies the LTS version. Since the entire
#    file in the kernel/configs repo is for a single kernel version,
#    the section is renamed as a "group", and the LTS version is
#    specified once at the top of the file with a tag of the form
#    <kernel minlts="x.y.z" />.
#  - The compatibility matrix understands all kernel config options as
#    tristate values. In reality however some kernel config options are
#    boolean. This script simply converts booleans to tristates so we
#    can avoid describing boolean values as tristates in hand-maintained
#    files.
#

from __future__ import print_function
import argparse
import os
import re
import sys

def fixup(args):
    source_f = open(args.input) or die ("Could not open %s" % args.input)

    # The first line of the conditional xml has the tag containing
    # the kernel min LTS version.
    line = source_f.readline()
    exp_re = re.compile(r"^<kernel minlts=\"(\d+).(\d+).(\d+)\"\s+/>")
    exp_match = re.match(exp_re, line)
    assert exp_match, "Malformatted kernel conditional config file.\n"

    major = exp_match.group(1)
    minor = exp_match.group(2)
    tiny = exp_match.group(3)

    if args.output_version:
        version_f = (open(args.output_version, "w+") or
                  die("Could not open version file"))
        version_f.write("{}.{}.{}".format(major, minor, tiny))
        version_f.close()

    if args.output_matrix:
        dest_f = (open(args.output_matrix, "w+") or
                  die("Could not open destination file"))
        dest_f.write("<compatibility-matrix version=\"1.0\" type=\"framework\">\n")

        # First <kernel> must not have <condition> for libvintf backwards compatibility.
        dest_f.write("<kernel version=\"{}.{}.{}\" />".format(major, minor, tiny))

        line = source_f.readline()
        while line:
            line = line.replace("<value type=\"bool\">",
                    "<value type=\"tristate\">")
            line = line.replace("<group>",
                    "<kernel version=\"{}.{}.{}\">".format(major, minor, tiny))
            line = line.replace("</group>", "</kernel>")
            dest_f.write(line)
            line = source_f.readline()

        dest_f.write("</compatibility-matrix>")
        dest_f.close()

    source_f.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', help='Input file', required=True)
    parser.add_argument('--output-matrix', help='Output compatibility matrix file')
    parser.add_argument('--output-version', help='Output version file')

    args = parser.parse_args()

    fixup(args)

    sys.exit(0)
