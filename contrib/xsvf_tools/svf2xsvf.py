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


# A python program to convert an SVF file to an XSVF file.  There is an
# option to include comments containing the source file line number from the origin
# SVF file before each outputted XSVF statement.
#
# We deviate from the XSVF spec in that we introduce a new command called
# XWAITSTATE which directly flows from the SVF RUNTEST command.  Unfortunately
# XRUNSTATE was ill conceived and is not used here.  We also add support for the
# three Lattice extensions to SVF: LCOUNT, LDELAY, and LSDR.  The xsvf file
# generated from this program is suitable for use with the xsvf player in
# OpenOCD with my modifications to xsvf.c.
#
# This program is written for python 3.0, and it is not easy to change this
# back to 2.x.  You may find it easier to use python 3.x even if that means
# building it.


import re
import sys
import struct


# There are both ---<Lexer>--- and ---<Parser>--- sections to this program


if len( sys.argv ) < 3:
    print("usage %s <svf_filename> <xsvf_filename>" % sys.argv[0])
    exit(1)


inputFilename = sys.argv[1]
outputFilename = sys.argv[2]

doCOMMENTs = True       # Save XCOMMENTs in the output xsvf file
#doCOMMENTs = False       # Save XCOMMENTs in the output xsvf file

# pick your file encoding
file_encoding = 'ISO-8859-1'
#file_encoding = 'utf-8'


xrepeat = 0             # argument to XREPEAT, gives retry count for masked compares


#-----< Lexer >---------------------------------------------------------------

StateBin = (RESET,IDLE,
    DRSELECT,DRCAPTURE,DRSHIFT,DREXIT1,DRPAUSE,DREXIT2,DRUPDATE,
    IRSELECT,IRCAPTURE,IRSHIFT,IREXIT1,IRPAUSE,IREXIT2,IRUPDATE) = range(16)

# Any integer index into this tuple will be equal to its corresponding StateBin value
StateTxt = ("RESET","IDLE",
    "DRSELECT","DRCAPTURE","DRSHIFT","DREXIT1","DRPAUSE","DREXIT2","DRUPDATE",
    "IRSELECT","IRCAPTURE","IRSHIFT","IREXIT1","IRPAUSE","IREXIT2","IRUPDATE")


(XCOMPLETE,XTDOMASK,XSIR,XSDR,XRUNTEST,hole0,hole1,XREPEAT,XSDRSIZE,XSDRTDO,
    XSETSDRMASKS,XSDRINC,XSDRB,XSDRC,XSDRE,XSDRTDOB,XSDRTDOC,
    XSDRTDOE,XSTATE,XENDIR,XENDDR,XSIR2,XCOMMENT,XWAIT,XWAITSTATE,
    LCOUNT,LDELAY,LSDR,XTRST) = range(29)

#Note: LCOUNT, LDELAY, and LSDR are Lattice extensions to SVF and provide a way to loop back
# and check a completion status, essentially waiting on a part until it signals that it is done.
# For example below: loop 25 times, each time through the loop do a LDELAY (same as a true RUNTEST)
# and exit loop when LSDR compares match.
"""
LCOUNT	25;
! Step to DRPAUSE give 5 clocks and wait for 1.00e+000 SEC.
LDELAY	DRPAUSE	5 TCK	1.00E-003 SEC;
! Test for the completed status. Match means pass.
! Loop back to LDELAY line if not match and loop count less than 25.
LSDR  1 TDI  (0)
        TDO  (1);
"""

#XTRST is an opcode Xilinx seemed to have missed and it comes from the SVF TRST statement.

LineNumber = 1

def s_ident(scanner, token): return ("ident", token.upper(), LineNumber)

def s_hex(scanner, token):
    global LineNumber
    LineNumber = LineNumber + token.count('\n')
    token = ''.join(token.split())
    return ("hex", token[1:-1], LineNumber)

def s_int(scanner, token): return ("int", int(token), LineNumber)
def s_float(scanner, token): return ("float", float(token), LineNumber)
#def s_comment(scanner, token): return ("comment", token, LineNumber)
def s_semicolon(scanner, token): return ("semi", token, LineNumber)

def s_nl(scanner,token):
    global LineNumber
    LineNumber = LineNumber + 1
    #print( 'LineNumber=', LineNumber, file=sys.stderr )
    return None

#2.00E-002

scanner = re.Scanner([
    (r"[a-zA-Z]\w*", s_ident),
#    (r"[-+]?[0-9]+[.]?[0-9]*([eE][-+]?[0-9]+)?", s_float),
    (r"[-+]?[0-9]+(([.][0-9eE+-]*)|([eE]+[-+]?[0-9]+))", s_float),
    (r"\d+", s_int),
    (r"\(([0-9a-fA-F]|\s)*\)", s_hex),
    (r"(!|//).*$", None),
    (r";", s_semicolon),
    (r"\n",s_nl),
    (r"\s*", None),
    ],
    re.MULTILINE
    )

# open the file using the given encoding
file = open( sys.argv[1], encoding=file_encoding )

# read all svf file input into string "input"
input = file.read()

file.close()

# Lexer:
# create a list of tuples containing (tokenType, tokenValue, LineNumber)
tokens = scanner.scan( input )[0]

input = None    # allow gc to reclaim memory holding file

#for tokenType, tokenValue, ln in tokens: print( "line %d: %s" % (ln, tokenType), tokenValue )


#-----<parser>-----------------------------------------------------------------

tokVal = tokType = tokLn = None

tup = iter( tokens )

def nextTok():
    """
    Function to read the next token from tup into tokType, tokVal, tokLn (linenumber)
    which are globals.
    """
    global tokType, tokVal, tokLn, tup
    tokType, tokVal, tokLn = tup.__next__()


class ParseError(Exception):
    """A class to hold a parsing error message"""
    def __init__(self, linenumber, token, message):
        self.linenumber = linenumber
        self.token = token
        self.message = message
    def __str__(self):
        global inputFilename
        return "Error in file \'%s\' at line %d near token %s\n %s" % (
                   inputFilename, self.linenumber, repr(self.token), self.message)


class MASKSET(object):
    """
    Class MASKSET holds a set of bit vectors, all of which are related, will all
    have the same length, and are associated with one of the seven shiftOps:
    HIR, HDR, TIR, TDR, SIR, SDR, LSDR. One of these holds a mask, smask, tdi, tdo, and a
    size.
    """
    def __init__(self, name):
        self.empty()
        self.name = name

    def empty(self):
        self.mask = bytearray()
        self.smask = bytearray()
        self.tdi = bytearray()
        self.tdo = bytearray()
        self.size = 0

    def syncLengths( self, sawTDI, sawTDO, sawMASK, sawSMASK, newSize ):
        """
        Set all the lengths equal in the event some of the masks were
        not seen as part of the last change set.
        """
        if self.size == newSize:
            return

        if newSize == 0:
            self.empty()
            return

        # If an SIR was given without a MASK(), then use a mask of all zeros.
        # this is not consistent with the SVF spec, but it makes sense because
        # it would be odd to be testing an instruction register read out of a
        # tap without giving a mask for it.  Also, lattice seems to agree and is
        # generating SVF files that comply with this philosophy.
        if self.name == 'SIR' and not sawMASK:
            self.mask = bytearray( newSize )

        if newSize != len(self.mask):
            self.mask = bytearray( newSize )
            if self.name == 'SDR':  # leave mask for HIR,HDR,TIR,TDR,SIR zeros
                for i in range( newSize ):
                    self.mask[i] = 1

        if newSize != len(self.tdo):
            self.tdo = bytearray( newSize )

        if newSize != len(self.tdi):
            self.tdi = bytearray( newSize )

        if newSize != len(self.smask):
            self.smask = bytearray( newSize )

        self.size = newSize
#-----</MASKSET>-----


def makeBitArray( hexString, bitCount ):
    """
    Converts a packed sequence of hex ascii characters into a bytearray where
    each element in the array holds exactly one bit. Only "bitCount" bits are
    scanned and these must be the least significant bits in the hex number. That
    is, it is legal to have some unused bits in the must significant hex nibble
    of the input "hexString". The string is scanned starting from the backend,
    then just before returning we reverse the array. This way the append()
    method can be used, which I assume is faster than an insert.
    """
    global tokLn
    a = bytearray()
    length = bitCount
    hexString = list(hexString)
    hexString.reverse()
    #print(hexString)
    for c in hexString:
        if length <= 0:
            break;
        c = int(c, 16)
        for mask in [1,2,4,8]:
            if length <= 0:
                break;
            length = length - 1
            a.append( (c & mask) != 0 )
    if length > 0:
        raise ParseError( tokLn, hexString, "Insufficient hex characters for given length of %d" % bitCount )
    a.reverse()
    #print(a)
    return a


def makeXSVFbytes( bitarray ):
    """
    Make a bytearray which is contains the XSVF bits which will be written
    directly to disk.  The number of bytes needed is calculated from the size
    of the argument bitarray.
    """
    bitCount = len(bitarray)
    byteCount = (bitCount+7)//8
    ba = bytearray( byteCount )
    firstBit = (bitCount % 8) - 1
    if firstBit == -1:
        firstBit = 7
    bitNdx = 0
    for byteNdx in range(byteCount):
        mask = 1<<firstBit
        byte = 0
        while mask:
            if bitarray[bitNdx]:
                byte |= mask;
            mask = mask >> 1
            bitNdx = bitNdx + 1
        ba[byteNdx] = byte
        firstBit = 7
    return ba


def writeComment( outputFile, shiftOp_linenum, shiftOp ):
    """
    Write an XCOMMENT record to outputFile
    """
    comment = "%s @%d\0" % (shiftOp, shiftOp_linenum)   # \0 is terminating nul
    ba = bytearray(1)
    ba[0] = XCOMMENT
    ba += comment.encode()
    outputFile.write( ba )


def combineBitVectors( trailer, meat, header ):
    """
    Combine the 3 bit vectors comprizing a transmission.  Since the least
    significant bits are sent first, the header is put onto the list last so
    they are sent first from that least significant position.
    """
    ret = bytearray()
    ret.extend( trailer )
    ret.extend( meat )
    ret.extend( header )
    return ret


def writeRUNTEST( outputFile, run_state, end_state, run_count, min_time, tokenTxt ):
    """
    Write the output for the SVF RUNTEST command.
    run_count - the number of clocks
    min_time - the number of seconds
    tokenTxt - either RUNTEST or LDELAY
    """
    # convert from secs to usecs
    min_time = int( min_time * 1000000)

    # the SVF RUNTEST command does NOT map to the XSVF XRUNTEST command.  Check the SVF spec, then
    # read the XSVF command.   They are not the same.  Use an XSVF XWAITSTATE to
    # implement the required behavior of the SVF RUNTEST command.
    if doCOMMENTs:
        writeComment( output, tokLn, tokenTxt )

    if tokenTxt == 'RUNTEST':
        obuf = bytearray(11)
        obuf[0] = XWAITSTATE
        obuf[1] = run_state
        obuf[2] = end_state
        struct.pack_into(">i", obuf, 3, run_count )  # big endian 4 byte int to obuf
        struct.pack_into(">i", obuf, 7, min_time )   # big endian 4 byte int to obuf
        outputFile.write( obuf )
    else:   # == 'LDELAY'
        obuf = bytearray(10)
        obuf[0] = LDELAY
        obuf[1] = run_state
        # LDELAY has no end_state
        struct.pack_into(">i", obuf, 2, run_count )  # big endian 4 byte int to obuf
        struct.pack_into(">i", obuf, 6, min_time )   # big endian 4 byte int to obuf
        outputFile.write( obuf )


output = open( outputFilename, mode='wb' )

hir = MASKSET('HIR')
hdr = MASKSET('HDR')
tir = MASKSET('TIR')
tdr = MASKSET('TDR')
sir = MASKSET('SIR')
sdr = MASKSET('SDR')


expecting_eof = True


# one of the commands that take the shiftParts after the length, the parse
# template for all of these commands is identical
shiftOps = ('SDR', 'SIR', 'LSDR', 'HDR', 'HIR', 'TDR', 'TIR')

# the order must correspond to shiftOps, this holds the MASKSETS.  'LSDR' shares sdr with 'SDR'
shiftSets = (sdr, sir, sdr, hdr, hir, tdr, tir )

# what to expect as parameters to a shiftOp, i.e. after a SDR length or SIR length
shiftParts = ('TDI', 'TDO', 'MASK', 'SMASK')

# the set of legal states which can trail the RUNTEST command
run_state_allowed = ('IRPAUSE', 'DRPAUSE', 'RESET', 'IDLE')

enddr_state_allowed = ('DRPAUSE', 'IDLE')
endir_state_allowed = ('IRPAUSE', 'IDLE')

trst_mode_allowed = ('ON', 'OFF', 'Z', 'ABSENT')

enddr_state = IDLE
endir_state = IDLE

frequency = 	1.00e+006 # HZ;

# change detection for xsdrsize and xtdomask
xsdrsize = -1           # the last one sent, send only on change
xtdomask = bytearray()  # the last one sent, send only on change


# we use a number of single byte writes for the XSVF command below
cmdbuf = bytearray(1)


# Save the XREPEAT setting into the file as first thing.
obuf = bytearray(2)
obuf[0] = XREPEAT
obuf[1] = xrepeat
output.write( obuf )


try:
    while 1:
        expecting_eof = True
        nextTok()
        expecting_eof = False
        # print( tokType, tokVal, tokLn )

        if tokVal in shiftOps:
            shiftOp_linenum = tokLn
            shiftOp = tokVal

            set = shiftSets[shiftOps.index(shiftOp)]

            # set flags false, if we see one later, set that one true later
            sawTDI = sawTDO = sawMASK = sawSMASK = False

            nextTok()
            if tokType != 'int':
                raise ParseError( tokLn, tokVal, "Expecting 'int' giving %s length, got '%s'" % (shiftOp, tokType) )
            length = tokVal

            nextTok()

            while tokVal != ';':
                if tokVal not in shiftParts:
                    raise ParseError( tokLn, tokVal, "Expecting TDI, TDO, MASK, SMASK, or ';'")
                shiftPart = tokVal

                nextTok()

                if tokType != 'hex':
                    raise ParseError( tokLn, tokVal, "Expecting hex bits" )
                bits = makeBitArray( tokVal, length )

                if shiftPart == 'TDI':
                    sawTDI = True
                    set.tdi = bits

                elif shiftPart == 'TDO':
                    sawTDO = True
                    set.tdo = bits

                elif shiftPart == 'MASK':
                    sawMASK = True
                    set.mask = bits

                elif shiftPart == 'SMASK':
                    sawSMASK = True
                    set.smask = bits

                nextTok()

            set.syncLengths( sawTDI, sawTDO, sawMASK, sawSMASK, length )

            # process all the gathered parameters and generate outputs here
            if shiftOp == 'SIR':
                if doCOMMENTs:
                    writeComment( output, shiftOp_linenum, 'SIR' )

                tdi = combineBitVectors( tir.tdi, sir.tdi, hir.tdi )
                if len(tdi) > 255:
                    obuf = bytearray(3)
                    obuf[0] = XSIR2
                    struct.pack_into( ">h", obuf, 1, len(tdi) )
                else:
                    obuf = bytearray(2)
                    obuf[0] = XSIR
                    obuf[1] = len(tdi)
                output.write( obuf )
                obuf = makeXSVFbytes( tdi )
                output.write( obuf )

            elif shiftOp == 'SDR':
                if doCOMMENTs:
                    writeComment( output, shiftOp_linenum, shiftOp )

                if not sawTDO:
                    # pass a zero filled bit vector for the sdr.mask
                    mask = combineBitVectors( tdr.mask, bytearray(sdr.size), hdr.mask )
                    tdi  = combineBitVectors( tdr.tdi,  sdr.tdi,  hdr.tdi )

                    if xsdrsize != len(tdi):
                        xsdrsize = len(tdi)
                        cmdbuf[0] = XSDRSIZE
                        output.write( cmdbuf )
                        obuf = bytearray(4)
                        struct.pack_into( ">i", obuf, 0, xsdrsize )  # big endian 4 byte int to obuf
                        output.write( obuf )

                    if xtdomask != mask:
                        xtdomask = mask
                        cmdbuf[0] = XTDOMASK
                        output.write( cmdbuf )
                        obuf = makeXSVFbytes( mask )
                        output.write( obuf )

                    cmdbuf[0] = XSDR
                    output.write( cmdbuf )
                    obuf = makeXSVFbytes( tdi )
                    output.write( obuf )

                else:
                    mask = combineBitVectors( tdr.mask, sdr.mask, hdr.mask )
                    tdi  = combineBitVectors( tdr.tdi,  sdr.tdi,  hdr.tdi )
                    tdo  = combineBitVectors( tdr.tdo,  sdr.tdo,  hdr.tdo )

                    if xsdrsize != len(tdi):
                        xsdrsize = len(tdi)
                        cmdbuf[0] = XSDRSIZE
                        output.write( cmdbuf )
                        obuf = bytearray(4)
                        struct.pack_into(">i", obuf, 0, xsdrsize )  # big endian 4 byte int to obuf
                        output.write( obuf )

                    if xtdomask != mask:
                        xtdomask = mask
                        cmdbuf[0] = XTDOMASK
                        output.write( cmdbuf )
                        obuf = makeXSVFbytes( mask )
                        output.write( obuf )

                    cmdbuf[0] = XSDRTDO
                    output.write( cmdbuf )
                    obuf = makeXSVFbytes( tdi )
                    output.write( obuf )
                    obuf = makeXSVFbytes( tdo )
                    output.write( obuf )
                    #print( "len(tdo)=", len(tdo), "len(tdr.tdo)=", len(tdr.tdo), "len(sdr.tdo)=", len(sdr.tdo), "len(hdr.tdo)=", len(hdr.tdo) )

            elif shiftOp == 'LSDR':
                if doCOMMENTs:
                    writeComment( output, shiftOp_linenum, shiftOp )

                mask = combineBitVectors( tdr.mask, sdr.mask, hdr.mask )
                tdi  = combineBitVectors( tdr.tdi,  sdr.tdi,  hdr.tdi )
                tdo  = combineBitVectors( tdr.tdo,  sdr.tdo,  hdr.tdo )

                if xsdrsize != len(tdi):
                    xsdrsize = len(tdi)
                    cmdbuf[0] = XSDRSIZE
                    output.write( cmdbuf )
                    obuf = bytearray(4)
                    struct.pack_into(">i", obuf, 0, xsdrsize )  # big endian 4 byte int to obuf
                    output.write( obuf )

                if xtdomask != mask:
                    xtdomask = mask
                    cmdbuf[0] = XTDOMASK
                    output.write( cmdbuf )
                    obuf = makeXSVFbytes( mask )
                    output.write( obuf )

                cmdbuf[0] = LSDR
                output.write( cmdbuf )
                obuf = makeXSVFbytes( tdi )
                output.write( obuf )
                obuf = makeXSVFbytes( tdo )
                output.write( obuf )
                #print( "len(tdo)=", len(tdo), "len(tdr.tdo)=", len(tdr.tdo), "len(sdr.tdo)=", len(sdr.tdo), "len(hdr.tdo)=", len(hdr.tdo) )

        elif tokVal == 'RUNTEST' or tokVal == 'LDELAY':
            # e.g. from lattice tools:
            # "RUNTEST	IDLE  	5 TCK	1.00E-003 SEC;"
            saveTok = tokVal
            nextTok()
            min_time = 0
            run_count = 0
            max_time = 600  # ten minutes
            if tokVal in run_state_allowed:
                run_state = StateTxt.index(tokVal)
                end_state = run_state  # bottom of page 17 of SVF spec
                nextTok()
            if tokType != 'int' and tokType != 'float':
                raise ParseError( tokLn, tokVal, "Expecting 'int' or 'float' after RUNTEST [run_state]")
            timeval = tokVal;
            nextTok()
            if tokVal != 'TCK' and tokVal != 'SEC' and tokVal != 'SCK':
                raise ParseError( tokLn, tokVal, "Expecting 'TCK' or 'SEC' or 'SCK' after RUNTEST [run_state] (run_count|min_time)")
            if tokVal == 'TCK' or tokVal == 'SCK':
                run_count = int( timeval )
            else:
                min_time = timeval
            nextTok()
            if tokType == 'int' or tokType == 'float':
                min_time = tokVal
                nextTok()
                if tokVal != 'SEC':
                    raise ParseError( tokLn, tokVal, "Expecting 'SEC' after RUNTEST [run_state] run_count min_time")
                nextTok()
            if tokVal == 'MAXIMUM':
                nextTok()
                if tokType != 'int' and tokType != 'float':
                    raise ParseError( tokLn, tokVal, "Expecting 'max_time' after RUNTEST [run_state] min_time SEC MAXIMUM")
                max_time = tokVal
                nextTok()
                if tokVal != 'SEC':
                    raise ParseError( tokLn, tokVal, "Expecting 'max_time' after RUNTEST [run_state] min_time SEC MAXIMUM max_time")
                nextTok()
            if tokVal == 'ENDSTATE':
                nextTok()
                if tokVal not in run_state_allowed:
                    raise ParseError( tokLn, tokVal, "Expecting 'run_state' after RUNTEST .... ENDSTATE")
                end_state = StateTxt.index(tokVal)
                nextTok()
            if tokVal != ';':
                raise ParseError( tokLn, tokVal, "Expecting ';' after RUNTEST ....")
            # print( "run_count=", run_count, "min_time=", min_time,
                # "max_time=", max_time, "run_state=", State[run_state], "end_state=", State[end_state] )
            writeRUNTEST( output, run_state, end_state, run_count, min_time, saveTok )

        elif tokVal == 'LCOUNT':
            nextTok()
            if tokType != 'int':
                raise ParseError( tokLn, tokVal, "Expecting integer 'count' after LCOUNT")
            loopCount = tokVal
            nextTok()
            if tokVal != ';':
                raise ParseError( tokLn, tokVal, "Expecting ';' after LCOUNT count")
            if doCOMMENTs:
                writeComment( output, tokLn, 'LCOUNT' )
            obuf = bytearray(5)
            obuf[0] = LCOUNT
            struct.pack_into(">i", obuf, 1, loopCount )  # big endian 4 byte int to obuf
            output.write( obuf )

        elif tokVal == 'ENDDR':
            nextTok()
            if tokVal not in enddr_state_allowed:
                raise ParseError( tokLn, tokVal, "Expecting 'stable_state' after ENDDR. (one of: DRPAUSE, IDLE)")
            enddr_state = StateTxt.index(tokVal)
            nextTok()
            if tokVal != ';':
                raise ParseError( tokLn, tokVal, "Expecting ';' after ENDDR stable_state")
            if doCOMMENTs:
                writeComment( output, tokLn, 'ENDDR' )
            obuf = bytearray(2)
            obuf[0] = XENDDR
            # Page 10 of the March 1999 SVF spec shows that RESET is also allowed here.
            # Yet the XSVF spec has no provision for that, and uses a non-standard, i.e.
            # boolean argument to XENDDR which only handles two of the 3 intended states.
            obuf[1] = 1 if enddr_state == DRPAUSE else 0
            output.write( obuf )

        elif tokVal == 'ENDIR':
            nextTok()
            if tokVal not in endir_state_allowed:
                raise ParseError( tokLn, tokVal, "Expecting 'stable_state' after ENDIR. (one of: IRPAUSE, IDLE)")
            endir_state = StateTxt.index(tokVal)
            nextTok()
            if tokVal != ';':
                raise ParseError( tokLn, tokVal, "Expecting ';' after ENDIR stable_state")
            if doCOMMENTs:
                writeComment( output, tokLn, 'ENDIR' )
            obuf = bytearray(2)
            obuf[0] = XENDIR
            # Page 10 of the March 1999 SVF spec shows that RESET is also allowed here.
            # Yet the XSVF spec has no provision for that, and uses a non-standard, i.e.
            # boolean argument to XENDDR which only handles two of the 3 intended states.
            obuf[1] = 1 if endir_state == IRPAUSE else 0
            output.write( obuf )

        elif tokVal == 'STATE':
            nextTok()
            ln = tokLn
            while tokVal != ';':
                if tokVal not in StateTxt:
                    raise ParseError( tokLn, tokVal, "Expecting 'stable_state' after STATE")
                stable_state = StateTxt.index( tokVal )

                if doCOMMENTs and ln != -1:
                    writeComment( output, ln, 'STATE' )
                    ln = -1     # save comment only once

                obuf = bytearray(2)
                obuf[0] = XSTATE
                obuf[1] = stable_state
                output.write( obuf )
                nextTok()

        elif tokVal == 'FREQUENCY':
            nextTok()
            if tokVal != ';':
                if tokType != 'int' and tokType != 'float':
                    raise ParseError( tokLn, tokVal, "Expecting 'cycles HZ' after FREQUENCY")
                frequency = tokVal
                nextTok()
                if tokVal != 'HZ':
                    raise ParseError( tokLn, tokVal, "Expecting 'HZ' after FREQUENCY cycles")
                nextTok()
                if tokVal != ';':
                    raise ParseError( tokLn, tokVal, "Expecting ';' after FREQUENCY cycles HZ")

        elif tokVal == 'TRST':
            nextTok()
            if tokVal not in trst_mode_allowed:
                raise ParseError( tokLn, tokVal, "Expecting 'ON|OFF|Z|ABSENT' after TRST")
            trst_mode = tokVal
            nextTok()
            if tokVal != ';':
                raise ParseError( tokLn, tokVal, "Expecting ';' after TRST trst_mode")
            if doCOMMENTs:
                writeComment( output, tokLn, 'TRST %s' % trst_mode )
            obuf = bytearray( 2 )
            obuf[0] = XTRST
            obuf[1] = trst_mode_allowed.index( trst_mode )  # use the index as the binary argument to XTRST opcode
            output.write( obuf )

        else:
            raise ParseError( tokLn, tokVal, "Unknown token '%s'" % tokVal)

except StopIteration:
    if not expecting_eof:
        print( "Unexpected End of File at line ", tokLn )

except ParseError as pe:
    print( "\n", pe )

finally:
    # print( "closing file" )
    cmdbuf[0] = XCOMPLETE
    output.write( cmdbuf )
    output.close()
