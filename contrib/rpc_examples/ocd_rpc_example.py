#!/usr/bin/env python3
"""
OpenOCD RPC example, covered by GNU GPLv3 or later
Copyright (C) 2014 Andreas Ortmann (ortmann@finf.uni-hannover.de)


Example output:
./ocd_rpc_example.py
echo says hi!

target state: halted
target halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x00000188 msp: 0x10000fd8

variable @ 0x10000000: 0x01c9c380

variable @ 0x10000000: 0xdeadc0de

memory (before): ['0xdeadc0de', '0x00000011', '0xaaaaaaaa', '0x00000023',
'0x00000042', '0x0000ffff']

memory (after): ['0x00000001', '0x00000000', '0xaaaaaaaa', '0x00000023',
'0x00000042', '0x0000ffff']
"""

import socket
import itertools

def strToHex(data):
    return map(strToHex, data) if isinstance(data, list) else int(data, 16)

def hexify(data):
    return "<None>" if data is None else ("0x%08x" % data)

def compareData(a, b):
    for i, j, num in zip(a, b, itertools.count(0)):
        if i != j:
            print("difference at %d: %s != %s" % (num, hexify(i), hexify(j)))


class OpenOcd:
    COMMAND_TOKEN = '\x1a'
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.tclRpcIp       = "127.0.0.1"
        self.tclRpcPort     = 6666
        self.bufferSize     = 4096

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __enter__(self):
        self.sock.connect((self.tclRpcIp, self.tclRpcPort))
        return self

    def __exit__(self, type, value, traceback):
        try:
            self.send("exit")
        finally:
            self.sock.close()

    def send(self, cmd):
        """Send a command string to TCL RPC. Return the result that was read."""
        data = (cmd + OpenOcd.COMMAND_TOKEN).encode("utf-8")
        if self.verbose:
            print("<- ", data)

        self.sock.send(data)
        return self._recv()

    def _recv(self):
        """Read from the stream until the token (\x1a) was received."""
        data = bytes()
        while True:
            chunk = self.sock.recv(self.bufferSize)
            data += chunk
            if bytes(OpenOcd.COMMAND_TOKEN, encoding="utf-8") in chunk:
                break

        if self.verbose:
            print("-> ", data)

        data = data.decode("utf-8").strip()
        data = data[:-1] # strip trailing \x1a

        return data

    def readVariable(self, address):
        raw = self.send("mdw 0x%x" % address).split(": ")
        return None if (len(raw) < 2) else strToHex(raw[1])

    def readMemory(self, wordLen, address, n):
        self.send("array unset output") # better to clear the array before
        self.send("mem2array output %d 0x%x %d" % (wordLen, address, n))

        output = [*map(int, self.send("return $output").split(" "))]
        d = dict([tuple(output[i:i + 2]) for i in range(0, len(output), 2)])

        return [d[k] for k in sorted(d.keys())]

    def writeVariable(self, address, value):
        assert value is not None
        self.send("mww 0x%x 0x%x" % (address, value))

    def writeMemory(self, wordLen, address, n, data):
        array = " ".join(["%d 0x%x" % (a, b) for a, b in enumerate(data)])

        self.send("array unset 1986ве1т") # better to clear the array before
        self.send("array set 1986ве1т { %s }" % array)
        self.send("array2mem 1986ве1т 0x%x %s %d" % (wordLen, address, n))

if __name__ == "__main__":

    def show(*args):
        print(*args, end="\n\n")

    with OpenOcd() as ocd:
        ocd.send("reset")

        show(ocd.send("capture { echo \"echo says hi!\" }")[:-1])
        show(ocd.send("capture \"halt\"")[:-1])

        # Read the first few words at the RAM region (put starting adress of RAM
        # region into 'addr')
        addr = 0x10000000

        value = ocd.readVariable(addr)
        show("variable @ %s: %s" % (hexify(addr), hexify(value)))

        ocd.writeVariable(addr, 0xdeadc0de)
        show("variable @ %s: %s" % (hexify(addr), hexify(ocd.readVariable(addr))))

        data = [1, 0, 0xaaaaaaaa, 0x23, 0x42, 0xffff]
        wordlen = 32
        n = len(data)

        read = ocd.readMemory(wordlen, addr, n)
        show("memory (before):", list(map(hexify, read)))

        ocd.writeMemory(wordlen, addr, n, data)

        read = ocd.readMemory(wordlen, addr, n)
        show("memory  (after):", list(map(hexify, read)))

        compareData(read, data)

        ocd.send("resume")
