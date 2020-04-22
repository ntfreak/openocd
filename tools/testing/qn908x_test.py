#!/usr/bin/env python3

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
Integration test module for the NXP QN908X family of chips.

To run these tests you need a QN908X board connected over SWD to a jlink
interface. The chip must be unlocked before all the tests run, for example,
after a mass erase.
"""

import random
import re
import struct
import tempfile
import unittest

import openocd_test


def SampleImage():
  """Returns a byte string with a valid image that allows SWD access."""
  base_addr = 0x01000000
  reset_addr = base_addr + 0x120

  # Point all ISR vectors to the reset_addr.
  vectors = [reset_addr + 1 for _ in range(69)]
  vectors[0] = 0x20000200  # Stack pointer
  vectors[7] = (- sum(vectors[:7])) % (1 << 32)
  vectors[8] = 0x000aa8ff  # Code Read Protection (CRP), all unprotected.
  vectors[9] = 0  # Image type, legacy.
  vectors[10] = 0  # Boot block pointer, unused in legacy mode.
  vectors[68] = 0  # ISP modes: all enabled.

  ret = struct.pack('<69I', *vectors)
  # 0 padding.
  ret += (reset_addr - base_addr - len(ret)) * b'\0'
  # Infinite loop.
  ret += struct.pack('<BB', 0xfe, 0xe7)
  ret += (4 - len(ret) % 4) * b'\0'
  return ret


class TestQN908x(openocd_test.OpenOCDTestCase):
  # Base address of the flash bank to test.
  BASE_ADDRESS = 0x01000000
  PAGE_SIZE = 2048
  NUM_PAGES = 256

  def setUp(self):
    super().setUp()
    self.startup_commands = [
        'source [find interface/jlink.cfg]',
        'transport select swd',
        'source [find target/qn908x.cfg]',
    ]

  @staticmethod
  def _load_protect_info(stdout):
    """Loads the output of "flash info" protection pages from stdout."""
    ret = {}
    for line in stdout:
      m = re.match(r'\s+# *([0-9]+):.*\) ([a-z ]*protect[a-z ]*)$', line)
      if m:
        ret[int(m.group(1))] = m.group(2)
    return ret

  def test_target_names(self):
    """Smoke test that the target is registered."""
    stdout, _ = self.OpenOCDEval(['target names'])
    self.assertEqual(stdout[-1], 'qn908x.cpu')

  def test_help(self):
    """All subcommands show up in the help."""
    stdout, _ = self.OpenOCDEval(['help qn908x'])
    for subcmd in ('disable_wdog', 'mass_erase', 'allow_brick'):
      self.assertTrue(any(line.startswith('  qn908x %s' % subcmd)
                          for line in stdout))

  def test_flash_banks(self):
    """The single bank of the chip is properly defined."""
    stdout, _ = self.OpenOCDEval(['init', 'flash banks'])
    if not stdout[-1]:
      stdout.pop()
    self.assertEqual(
        stdout[-1],
        ('#0 : qn908x.flash (qn908x) at 0x%.8x, size 0x00000000, buswidth 0, '
         'chipwidth 0' % self.BASE_ADDRESS))

  def test_reset_init(self):
    """Resetting and halting the device effectively halts it."""
    stdout, _ = self.OpenOCDEval([
        'init', 'reset init', 'echo [qn908x.cpu curstate]'])
    self.assertEqual(stdout[-1], 'halted')

  def test_mass_erase(self):
    """Test that we can mass erase the chip."""
    locks_file = tempfile.NamedTemporaryFile()
    stdout, _ = self.OpenOCDEval([
        'init',
        'reset halt',
        'qn908x mass_erase',
        'flash info 0',
        'dump_image "%s" 0x%.8x 0x%x' % (
            locks_file.name, self.BASE_ADDRESS + 0x7f800, 0x800),
    ])
    # None of the pages should be protected after a mass erase.
    protect = self._load_protect_info(stdout)
    for i in range(self.NUM_PAGES):
      self.assertEqual(protect[i], 'not protected')
    # Check the status of the lock on the flash.
    with open(locks_file.name, 'rb') as f:
      locks_data = f.read()
    # All the pages are not protected.
    self.assertTrue(all(b == 0xff for b in locks_data[:0x20]))
    # SWD RAM and FLASH access is allowed unless "keep_lock" is passed.
    self.assertEqual(0x01, locks_data[0x20])

  def test_mass_erase_keep_lock(self):
    """Test that mass_erase keep_lock doesn't write to the last page."""
    locks_file = tempfile.NamedTemporaryFile()
    self.OpenOCDEval([
        'init',
        'reset halt',
        'qn908x mass_erase keep_lock',
        'dump_image "%s" 0x%.8x 0x%x' % (
            locks_file.name, self.BASE_ADDRESS + 0x7f800, 0x800),
    ])
    # Check the status of the lock on the flash.
    with open(locks_file.name, 'rb') as f:
      locks_data = f.read()
    # All the pages are not protected.
    self.assertTrue(all(b == 0xff for b in locks_data[:0x20]))
    # SWD RAM and FLASH access is restricted when "keep_lock" is passed.
    self.assertEqual(0xff, locks_data[0x20])

  def test_mass_erase_protected(self):
    """Test that we can mass erase a protected chip."""
    data = SampleImage()
    data_file = tempfile.NamedTemporaryFile(mode='wb')
    with open(data_file.name, 'wb') as f:
      f.write(data)

    erased_file = tempfile.NamedTemporaryFile()

    self.OpenOCDEval([
        'init',
        'reset halt',
        'qn908x mass_erase',
        'flash protect 0 %d %d on' % (50, 60),
        # Pages 50 to 60 are protected here.
    ])

    stdout, _ = self.OpenOCDEval([
        'init',
        'reset halt',
        'flash write_image "%s" 0x%.8x' % (data_file.name, self.BASE_ADDRESS),
        'reset halt',
        'qn908x mass_erase',
        'flash info 0',
        'dump_image "%s" 0x%.8x 0x%x' % (
            erased_file.name, self.BASE_ADDRESS, 4096),
    ])
    # None of the pages should be protected after a mass erase.
    protect = self._load_protect_info(stdout)
    for i in range(self.NUM_PAGES):
      self.assertEqual(protect[i], 'not protected')

    # Check that the data was actually erased from the first pages.
    with open(erased_file.name, 'rb') as f:
      erased_data = f.read()
    self.assertTrue(all(b == 0xff for b in erased_data))

  def test_protect(self):
    """Protecting and unprotecting a page."""
    start = 61
    count = 7

    random.seed(1234)
    data = bytearray(
        random.getrandbits(8) for _ in range(count * self.PAGE_SIZE))
    data_file = tempfile.NamedTemporaryFile(mode='wb')
    with open(data_file.name, 'wb') as f:
      f.write(data)

    address = self.BASE_ADDRESS + start * self.PAGE_SIZE
    stdout, _ = self.OpenOCDEval([
        'init',
        'reset halt',
        'flash erase_address pad unlock 0x%.8x 0x%x' %
           (address, count * self.PAGE_SIZE),
        'flash protect 0 %d %d on' % (start, start + count - 1),
        'flash info 0',
    ])

    # The selected region must be marked as protected.
    protect = self._load_protect_info(stdout)
    for i in range(start, start + count):
      self.assertEqual(protect[i], 'protected')

    # We will be attempting to write data to the protected area which should
    # fail.
    stdout, _ = self.OpenOCDEval([
        'init',
        'halt',
        'flash write_image "%s" 0x%.8x' % (data_file.name, address),
    ], returncode=1)
    # Writing to protected memory would fail and set the INT_STAT register.
    self.assertTrue(any(line.startswith('Error: INT_STAT status error:')
                        for line in stdout))

  def DISABLED_test_protect_password(self):
    """Test that we can unprotect the last page with the password."""
    # TODO: openocd doesn't support unprotecting the last page. This requires to
    # run code from the RAM to override the lock in the last page, otherwise we
    # need to do a mass erase from SWD.
    count = 5
    start = self.NUM_PAGES - count

    # The last 5 pages should be protected after this.
    stdout, _ = self.OpenOCDEval([
        'init',
        'reset halt',
        'flash protect 0 %d %d on' % (start, start + count - 1),
        'flash info 0',
    ])
    protect = self._load_protect_info(stdout)
    for i in range(start, start + count):
      self.assertEqual(protect[i], 'protected')

    lucky_page = start + 2
    # Unprotecting a page in that group of 5 should only unprotect that one,
    # even though the driver needs to erase the last page.
    stdout, _ = self.OpenOCDEval([
        'init',
        'halt',
        'flash info 0',
        'flash protect 0 %d %d off' % (lucky_page, lucky_page),
        'flash info 0',
    ])
    protect = self._load_protect_info(stdout)
    for i in range(start, start + count):
      self.assertEqual(
          protect[i], 'protected' if i != lucky_page else 'not protected')

  def test_flash(self):
    """End-to-end flash and erase."""
    random.seed(1234)
    data = SampleImage() + bytearray(
        random.getrandbits(8) for _ in range(12340))
    data_file = tempfile.NamedTemporaryFile(mode='wb')
    with open(data_file.name, 'wb') as f:
      f.write(data)

    erased_file = tempfile.NamedTemporaryFile()
    readback_file = tempfile.NamedTemporaryFile()

    # The SampleImage prefix has a valid legacy checksum and CRP, so we can
    # flash it a address 0.
    for offset in (0, 0x5230):
      address = self.BASE_ADDRESS + offset

      self.OpenOCDEval([
          'init',
          'reset halt',
          # Erase the region manually before flashing.
          'flash erase_address pad unlock 0x%.8x 0x%x' % (address, len(data)),
          'dump_image "%s" 0x%.8x 0x%x' % (
              erased_file.name, address, len(data)),
          'flash write_image "%s" 0x%.8x' % (data_file.name, address),
          'dump_image "%s" 0x%.8x 0x%x' % (
              readback_file.name, address, len(data)),
      ])

      with open(erased_file.name, 'rb') as f:
        erased_data = f.read()
      self.assertEqual(len(erased_data), len(data))
      self.assertTrue(all(b == 0xff for b in erased_data))

      with open(readback_file.name, 'rb') as f:
        readback_data = f.read()
      self.assertEqual(len(readback_data), len(data))
      self.assertEqual(readback_data, data)

  def test_verify(self):
    """Test that verification works on a valid image with valid checksum"""
    random.seed(1234)
    # For testing the verification process it is important to use a somewhat
    # large image so we check that the disable_wdog indeed works as expected.
    data = SampleImage() + bytearray(
        random.getrandbits(8) for _ in range(12340))
    data_file = tempfile.NamedTemporaryFile(mode='wb')
    with open(data_file.name, 'wb') as f:
      f.write(data)

    self.OpenOCDEval([
        'init',
        'reset halt',
        'qn908x disable_wdog',
        # Erase the region manually before flashing.
        'flash write_image erase "%s" 0x%.8x' % (
            data_file.name, self.BASE_ADDRESS),
        'verify_image "%s" 0x%.8x' % (data_file.name, self.BASE_ADDRESS),
    ])


if __name__ == '__main__':
  unittest.main()
