#!/usr/bin/env python3
#
# Copyright (C) 2020 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""
Freeze kernel configs for a compatibility matrix release.
"""

import argparse
import datetime
import os
import shutil

import subprocess

def check_call(*args, **kwargs):
    print(args[0])
    subprocess.check_call(*args, **kwargs)

def replace_configs_module_name(new_release, file_path):
    check_call("sed -i'' -E 's/\"kernel_config_current_([0-9.]*)\"/\"kernel_config_{}_\\1\"/g' {}"
                .format(new_release, file_path), shell=True)

class Freeze(object):
    def __init__(self, cmdline_args):
        top = os.environ["ANDROID_BUILD_TOP"]
        self.new_release = cmdline_args.name
        self.configs_dir = os.path.join(top, "kernel/configs")
        self.new_release_dir = os.path.join(self.configs_dir, self.new_release)
        self.interfaces_dir = os.path.join(top, "hardware/interfaces")
        self.versions = [e for e in os.listdir(self.configs_dir) if e.startswith("android-")]
        self.bugline = "Bug: {}\n".format(cmdline_args.bug) if cmdline_args.bug else ""

    def run(self):
        self.move_configs()
        self.freeze_configs_in_matrices()
        self.create_current()
        self.print_summary()

    def move_configs(self):
        os.makedirs(self.new_release_dir, exist_ok=False)
        for version in self.versions:
            src = os.path.join(self.configs_dir, version)
            dst = os.path.join(self.new_release_dir, version)
            shutil.move(src, dst)
            for file_name in os.listdir(dst):
                abs_path = os.path.join(dst, file_name)
                if not os.path.isfile(abs_path):
                    continue
                year = datetime.datetime.now().year
                check_call("sed -i'' -E 's/Copyright \\(C\\) [0-9]{{4,}}/Copyright (C) {}/g' {}".format(year, abs_path), shell=True)
                replace_configs_module_name(self.new_release, abs_path)

        check_call('git -C {} add android-* {}'.format(self.configs_dir, self.new_release), shell=True)
        check_call('git -C {} commit -m "Freeze kernel configs for {}.\n\n{}Test: builds"'.format(self.configs_dir, self.new_release, self.bugline), shell=True)

    def freeze_configs_in_matrices(self):
        matrices_soong = "compatibility_matrices/Android.bp"
        matrices_soong_abs_path = os.path.join(self.interfaces_dir, matrices_soong)
        replace_configs_module_name(self.new_release, matrices_soong_abs_path)

        check_call('git -C {} add {}'.format(self.interfaces_dir, matrices_soong), shell=True)
        check_call('git -C {} commit -m "Freeze kernel configs for {}.\n\n{}Test: builds"'.format(self.interfaces_dir, self.new_release, self.bugline), shell=True)

    def create_current(self):
        check_call('git -C {} checkout HEAD~ -- android-*'.format(self.configs_dir), shell=True)
        check_call('git -C {} add android-*'.format(self.configs_dir), shell=True)
        check_call('git -C {} commit -m "Create kernel configs for current.\n\n{}Test: builds"'.format(self.configs_dir, self.bugline), shell=True)

    def print_summary(self):
        print("*** Please submit these changes to {} branch: ***".format(self.new_release))
        check_call('git -C {} show -s --format=%B HEAD~1'.format(self.configs_dir), shell=True)
        check_call('git -C {} show -s --format=%B HEAD'.format(self.interfaces_dir), shell=True)
        print("*** Please submit these changes to master branch: ***")
        check_call('git -C {} show -s --format=%B HEAD'.format(self.configs_dir), shell=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', type=str, help='name of the directory (e.g. r)')
    parser.add_argument('--bug', type=str, nargs='?', help='Bug number')
    cmdline_args = parser.parse_args()

    Freeze(cmdline_args).run()

if __name__ == '__main__':
    main()
