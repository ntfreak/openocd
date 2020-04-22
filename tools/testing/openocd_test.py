# Copyright (C) 2020 iosabi
# iosabi <iosabi@protonmail.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
Testing helper classes for running OpenOCD target unittest and integration test.

The "openocd" binary under test can be set by passing the OPENOCD environment
variable with the path to the openocd binary. It is also desirable to pass a
search path to using the OPENOCD_SEARCH path. Otherwise, the default paths are
those corresponding to the current checked out tree and locally compiled code.
"""

import os
import subprocess
import unittest


class OpenOCDTestCase(unittest.TestCase):
  def setUp(self):
    base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(
        __file__))))
    self.openocd_bin = os.environ.get(
        'OPENOCD', os.path.join(base_dir, 'src', 'openocd'))
    self.openocd_search = os.environ.get(
        'OPENOCD_SEARCH', os.path.join(base_dir, 'tcl'))
    self.startup_commands = []

  def OpenOCDEval(self, commands, returncode=0):
    cmd = [
        self.openocd_bin, '--search', self.openocd_search, '-l', '/dev/stdout']
    for arg in self.startup_commands + commands + ['shutdown']:
      cmd.append('-c')
      cmd.append(arg)
    ret = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=30)
    stdout = ret.stdout.decode('utf-8').rstrip('\n').split('\n')
    stderr = ret.stderr.decode('utf-8').rstrip('\n').split('\n')
    # Remove the 'shutdown command invoked' message from the output.
    if stdout and stdout[-1] == 'shutdown command invoked':
      stdout.pop()
    error_msg = '\nRunning %r\n--- stdout:\n%s\n--- stderr:\n%s\n' % (
        cmd, '\n'.join(stdout), '\n'.join(stderr))
    self.assertEqual(ret.returncode, returncode, error_msg)
    return stdout, stderr
