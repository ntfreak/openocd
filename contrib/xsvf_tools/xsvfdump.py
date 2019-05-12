#!/usr/bin/python3.0

# Copyright 2008, SoftPLC Corporation  http://softplc.com
# Dick Hollenbeck dick@softplc.com

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, you may find one here:
# http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
# or you may search the http://www.gnu.org website for the version 2 license,
# or you may write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA

# Dump an Xilinx XSVF file to stdout

# This program is written for python 3.0, and it is not easy to change this
# back to 2.x.  You may find it easier to use python 3.x even if that means
# building it.


import sys
import struct


LABEL = "A script to dump an XSVF file to stdout"


Xsdrsize = 0


(XCOMPLETE,XTDOMASK,XSIR,XSDR,XRUNTEST,hole0,hole1,XREPEAT,XSDRSIZE,XSDRTDO,
    XSETSDRMASKS,XSDRINC,XSDRB,XSDRC,XSDRE,XSDRTDOB,XSDRTDOC,
    XSDRTDOE,XSTATE,XENDIR,XENDDR,XSIR2,XCOMMENT,XWAIT,XWAITSTATE,
    LCOUNT,LDELAY,LSDR,XTRST) = range(29)


(RESET,IDLE,
    DRSELECT,DRCAPTURE,DRSHIFT,DREXIT1,DRPAUSE,DREXIT2,DRUPDATE,
    IRSELECT,IRCAPTURE,IRSHIFT,IREXIT1,IRPAUSE,IREXIT2,IRUPDATE) = range(16)


State = ("RESET","IDLE",
    "DRSELECT","DRCAPTURE","DRSHIFT","DREXIT1","DRPAUSE","DREXIT2","DRUPDATE",
    "IRSELECT","IRCAPTURE","IRSHIFT","IREXIT1","IRPAUSE","IREXIT2","IRUPDATE")


trst_mode_allowed = ('ON', 'OFF', 'Z', 'ABSENT')


Setsdrmasks = 0
SetsdrmasksOnesCount = 0

def ReadSDRMASKS( f, len ):
    global Setsdrmasks, SetsdrmasksOnesCount
    byteCount = (len+7)//8
    Setsdrmasks = f.read( byteCount )
    ls = []
    SetsdrmasksOnesCount = 0
    for b in Setsdrmasks:
        ls.append( "%x" % ((b & 0xf0) >> 4) )
        ls.append( "%x" % ( b & 0x0f ) )
        for i in range(8):
            if b & (1<<i):
                SetsdrmasksOnesCount = SetsdrmasksOnesCount +1
    return ''.join(ls)


def bytes2hexString( f, len ):
    byteCount = (len+7)//8
    bytebuf = f.read( byteCount )
    ls = []
    for b in bytebuf:
        ls.append( "%x" % ((b & 0xf0) >> 4) )
        ls.append( "%x" % ( b & 0x0f ) )
    return ''.join(ls)


def ReadByte( f ):
    """Read a byte from a file and return it as an int in least significant 8 bits"""
    b = f.read(1)
    if b:
        return 0xff & b[0];
    else:
        return -1


def ShowState( state ):
    """return the given state int as a state string"""
    #return "0x%02x" % state # comment this out to get textual state form
    global State
    if 0 <= state <= IRUPDATE:
        return State[state]
    else:
        return "Unknown state 0x%02x" % state


def ShowOpcode( op, f ):
    """return the given byte as an opcode string"""
    global Xsdrsize
    if op == XCOMPLETE:
        print("XCOMPLETE")

    elif op == XTDOMASK:
        buf = bytes2hexString( f, Xsdrsize )
        print("XTDOMASK 0x%s" % buf)

    elif op == XSIR:
        len = ReadByte( f )
        buf = bytes2hexString( f, len )
        print("XSIR 0x%02X 0x%s" % (len, buf))

    elif op == XSDR:
        tdi = bytes2hexString( f, Xsdrsize )
        print("XSDR 0x%s" % tdi)

    elif op == XRUNTEST:
        len = struct.unpack( '>i', f.read(4) )[0]
        print("XRUNTEST 0x%08X" % len)

    elif op == XREPEAT:
        len = ReadByte( f )
        print("XREPEAT 0x%02X" % len)

    elif op == XSDRSIZE:
        Xsdrsize = struct.unpack( '>i', f.read(4) )[0]
        #print("XSDRSIZE 0x%08X" % Xsdrsize, file=sys.stderr )
        print("XSDRSIZE 0x%08X %d" % (Xsdrsize, Xsdrsize) )

    elif op == XSDRTDO:
        tdi = bytes2hexString( f, Xsdrsize )
        tdo = bytes2hexString( f, Xsdrsize )
        print("XSDRTDO 0x%s 0x%s" % (tdi, tdo) )

    elif op == XSETSDRMASKS:
        addrmask = bytes2hexString( f, Xsdrsize )
        datamask = ReadSDRMASKS( f, Xsdrsize )
        print("XSETSDRMASKS 0x%s 0x%s" % (addrmask, datamask) )

    elif op == XSDRINC:
        startaddr = bytes2hexString( f, Xsdrsize )
        len = ReadByte(f)
        print("XSDRINC 0x%s 0x%02X" % (startaddr, len), end='' )
        for numTimes in range(len):
            data = bytes2hexString( f, SetsdrmasksOnesCount)
            print(" 0x%s" % data )
        print() # newline

    elif op == XSDRB:
        tdi = bytes2hexString( f, Xsdrsize )
        print("XSDRB 0x%s" % tdi )

    elif op == XSDRC:
        tdi = bytes2hexString( f, Xsdrsize )
        print("XSDRC 0x%s" % tdi )

    elif op == XSDRE:
        tdi = bytes2hexString( f, Xsdrsize )
        print("XSDRE 0x%s" % tdi )

    elif op == XSDRTDOB:
        tdo = bytes2hexString( f, Xsdrsize )
        print("XSDRTDOB 0x%s" % tdo )

    elif op == XSDRTDOC:
        tdi = bytes2hexString( f, Xsdrsize )
        tdo = bytes2hexString( f, Xsdrsize )
        print("XSDRTDOC 0x%s 0x%s" % (tdi, tdo) )

    elif op == XSDRTDOE:
        tdi = bytes2hexString( f, Xsdrsize )
        tdo = bytes2hexString( f, Xsdrsize )
        print("XSDRTDOE 0x%s 0x%s" % (tdi, tdo) )

    elif op == XSTATE:
        b = ReadByte(f)
        print("XSTATE %s" % ShowState(b))

    elif op == XENDIR:
        b = ReadByte( f )
        print("XENDIR %s" % 'IRPAUSE' if b==1 else 'IDLE')

    elif op == XENDDR:
        b = ReadByte( f )
        print("XENDDR %s" % 'DRPAUSE' if b==1 else 'IDLE')

    elif op == XSIR2:
        len = struct.unpack( '>H', f.read(2) )[0]
        buf = bytes2hexString( f, len )
        print("XSIR2 0x%04X 0x%s" % (len, buf))

    elif op == XCOMMENT:
        cmt = []
        while 1:
            b = ReadByte(f)
            if b == 0:          # terminating nul
                break;
            cmt.append( chr(b) )
        print("XCOMMENT \"%s\"" % ''.join(cmt)  )

    elif op == XWAIT:
        run_state = ReadByte(f)
        end_state = ReadByte(f)
        useconds  = struct.unpack( '>i', f.read(4) )[0]
        print("XWAIT %s %s" % (ShowState(run_state), ShowState(end_state)), useconds)

    elif op == XWAITSTATE:
        run_state = ReadByte(f)
        end_state = ReadByte(f)
        clocks    = struct.unpack( '>i', f.read(4) )[0]
        useconds  = struct.unpack( '>i', f.read(4) )[0]
        print("XWAITSTATE %s %s CLOCKS=%d USECS=%d" % (ShowState(run_state), ShowState(end_state), clocks, useconds) )

    elif op == LCOUNT:
        loop_count = struct.unpack( '>i', f.read(4) )[0]
        print("LCOUNT", loop_count )

    elif op == LDELAY:
        run_state = ReadByte(f)
        clocks    = struct.unpack( '>i', f.read(4) )[0]
        useconds  = struct.unpack( '>i', f.read(4) )[0]
        print("LDELAY %s CLOCKS=%d USECS=%d" % (ShowState(run_state), clocks, useconds) )

    elif op == LSDR:
        tdi = bytes2hexString( f, Xsdrsize )
        tdo = bytes2hexString( f, Xsdrsize )
        print("LSDR 0x%s 0x%s" % (tdi, tdo) )

    elif op == XTRST:
        # the argument is a single byte and it is the index into "trst_mode_allowed"
        trst_mode = ReadByte(f)
        if trst_mode <= 3:
            print("TRST %s" % trst_mode_allowed[trst_mode] )
        else:
            print("TRST 0x%02X" % trst_mode );

    else:
        print("UNKNOWN op 0x%02X %d" % (op, op))
        exit(1)


def main():

    if len( sys.argv ) < 2:
        print("usage %s <xsvf_filename>" % sys.argv[0])
        exit(1)

    f = open( sys.argv[1], 'rb' )

    opcode = ReadByte( f )
    while opcode != -1:
        # print the position within the file, then the command
        print( "%d: " % f.tell(), end='' )
        ShowOpcode( opcode, f )
        opcode = ReadByte(f)


if __name__ == "__main__":
    main()
