#!/bin/perl
#***************************************************************************
#*   Copyright (C) 2008 Lou Deluxe                                         *
#*   lou.openocd012@fixit.nospammail.net                                   *
#*                                                                         *
#*   This program is free software; you can redistribute it and/or modify  *
#*   it under the terms of the GNU General Public License as published by  *
#*   the Free Software Foundation; either version 2 of the License, or     *
#*   (at your option) any later version.                                   *
#*                                                                         *
#*   This program is distributed in the hope that it will be useful,       *
#*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
#*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
#*   GNU General Public License for more details.                          *
#*                                                                         *
#*   You should have received a copy of the GNU General Public License     *
#*   along with this program; if not, write to the                         *
#*   Free Software Foundation, Inc.,                                       *
#*   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
#***************************************************************************

# A rudimentary assembler for DTC code.
# It is not robust, by any means, but it gets the job done.

{package DTC_as;

my($i);	# for later loop to generate reverse lookup

sub new {
	my($self) = bless{};

	$self->{'pagewidth'} = 60;
	$self;
}



%status_bit_arg = (
	'STOP' => 0x01,
	'ERROR' => 0x02,
);

%cp_arg = (
	'A=>X' => 0x00,
	'A<X' => 0x01,
	'CARRY' => 0x02,
	'ALWAYS' => 0x03,
	'ADR_BUFFER0=>CMP0' => 0x04,
	'ADR_BUFFER0<CMP0' => 0x05,
	'ADR_BUFFER1=>CMP1' => 0x06,
	'ADR_BUFFER1<CMP1' => 0x07,
);

%shift_unit_arg = (
	'CARD' => 0x00,
	'MPEG' => 0x08,
);

%shift_pin_arg = (
	'PIN0=>IN' => 0x00,
	'PIN1=>IN' => 0x04,
	'OUT=>PIN0' => 0x01,
	'OUT=>PIN1' => 0x03,
);

@ld_arg = (
	'<Y>',
	'X',
	'Y',
	'MASK',
	'ADR_BUFFER00',
	'ADR_BUFFER01',
	'ADR_BUFFER10',
	'ADR_BUFFER11',
	'CMP00',
	'CMP01',
	'CMP10',
	'CMP11',
	'DATA_FLASH',
	'CTRL_FCI',
	'CTRL_CARD',
	'CTRL_MPEG',
	'DR_PARALLEL',
	'DDR_PARALLEL',
	'OR_PARALLEL',
	'DR_CARD',
	'DDR_CARD',
	'OR_CARD',
	'SHIFT_CARD',
	'DR_MPEG',
	'DDR_MPEG',
	'OR_MPEG',
	'SHIFT_MPEG',
	'DATA_BUFFER0',
	'DATA_BUFFER1',
	'ECC_CRC',
	'TMP_ECC',
	'BUFFER_MNGT'
);

for($i = 0; $i < @ld_arg; $i++) {
	$ld_arg{$ld_arg[$i]} = $i;
}


# ADDER8 / SUB8
sub alu8 {
	my($self) = shift;
	my($operand, $i) = shift;

	if(defined($ld_arg{$operand})) {
		$i = $ld_arg{$operand};

		if($i > 0x00 && $i < 0x04) {
			return(($i - 0x01) << 3);
		}
	}

	return undef;
}

# ADDER16 / SUB16
sub alu16 {
	my($self) = shift;
	my($operand, $i) = shift;

	$operand .= '0';

	if(defined($ld_arg{$operand})) {
		$i = $ld_arg{$operand};

		if($i > 0x03 && $i < 0x0c) {
			return(($i - 0x04) << 2);
		}
	}

	return undef;
}


# BSET / BCLR
sub bsetorclr {
	my($self) = shift;
	my($ret);
	
	if(@_ < 1) {
		return undef;
	}
	$ret = $_[0];

	if(($ret < 0) || ($ret > 3)) {
		return undef;
	}

	return $ret;
}


# Opcode lookup table
%op = (
	'NOP' => [
		0x0,
	],
	'SEC' => [
		0x1,
	],
	'CLC' => [
		0x2,
	],
	'RET' => [
		0x3,
	],
	'STATUS' => [
		0x4,
		sub {
			my($self) = shift;
			my($ret, $i);

			for $i (@_) {
				if(!defined($status_bit_arg{"\U$i"})) {
					return undef;
				}

				$ret |= $status_bit_arg{"\U$i"};
			}
			if($ret < 1) {
				return undef;
			}

			return $ret;
		}
	],
	'CP' => [
		0x8,
		sub {
			my($self) = shift;
			if((@_ != 1) || (!defined($cp_arg{"\U$_[0]"}))) {
				return undef;
			}
			return($cp_arg{"\U$_[0]"});
		}
	],
	'SHIFT' => [
		0x10,
		sub {
			my($self) = shift;
			my($ret, $i);
			
			if((@_ < 2) || (!defined($shift_unit_arg{"\U$_[0]"}))) {
				return undef;
			}
			$ret = $shift_unit_arg{"\U$_[0]"};
			shift;

			for $i (@_) {
				if(!defined($shift_pin_arg{"\U$i"})) {
					return undef;
				}

				$ret |= $shift_pin_arg{"\U$i"};
			}

			return $ret;
		}
	],
	'SUB8' => [
		0x24,
		\&alu8
	],
	'ADDER8' => [
		0x25,
		\&alu8
	],
	'SUB16' => [
		0x26,
		\&alu16
	],
	'ADDER16' => [
		0x27,
		\&alu16
	],
	'BCLR' => [
		0x28,
		\&bsetorclr
	],
	'BSET' => [
		0x38,
		\&bsetorclr
	],
	'REVERSE' => [
		0x30,
	],
	'XOR' => [
		0x31,
	],
	'AND' => [
		0x32,
	],
	'EXCHANGE' => [
		0x33,
	],
	'DECY' => [
		0x3c,
	],
	'INCY' => [
		0x3d,
	],
	'JP' => [
		0x40,
		sub {
			my($self) = shift;
			my($i);

			if(@_ != 1) {
				return undef;
			}
			$i = $_[0];
			if(!defined($self->{'label'}{$i})) {
				$i =~ s/^://o;
				if(!defined($self->{'label'}{$i})) {
					# not a defined label
					undef $i;
				}
			}

			if(defined($i)) {
				$i = $self->{'label'}{$i} - $self->{'pc'};
			} else {
				$i = $_[0];
			}

			if($i =~ m/^([+-]?\d+)$/) {
				$i = 0 + $1;
				if(($i > 31) || ($i < -31)) {
					warn "relative jump ($i) out of range";
					return undef;
				}
				if($i < 0) {
					return(0x20 - $1);
				} else {
					return(0x00 + $1);
				}
			}

			return undef;
		}
	],
	'BRANCH' => [
		0x60,
	],
	'LD' => [
		0x80,
		sub {
			my($self) = shift;
			my($i);

#			print STDERR join(", ", LD, @_), "\n";

			if(@_ == 1) {
				$_[1] = 'A';
			}
			if(@_ != 2) {
				return undef;
			}

				
			if($_[0] =~ m/^([ML])S[BN]$/o) {
				# MSB/LSB aka MSN/LSN
				if($1 eq 'L') {
					$_[0] = 'A.L';
				} else {
					$_[0] = 'A.H';
				}
			}
			if($_[0] =~ m/^A\.([LH])$/o) {
				# A.L/A.H
				my($islsb) = ($1 eq 'L') ? 1 : 0;
				$i = $_[1];
				if($i =~ s/^0x([0-9a-fA-F])$/hex($1)/e) {
#					print "$i looks hex\n";
				} elsif($i =~ m/^\d+$/) {
#					print "$i looks decimal\n";
				} elsif(defined($self->{'label'}{$i})) {
#					print "label match for $i ($self->{'label'}{$i})\n";
					$i = $self->{'label'}{$i};
#					print "\$i=$i\n";
#					print "\$islsb=$islsb\n";
					if($islsb) {
						$i = ($i & 0xf);
					} else {
						$i = ($i >> 4) & 0xf;
					}
#					print "\$i=$i\n";
				} else {
					print "no label match for $i\n";
					return undef;
				}
				if(($i < 0) || ($i > 0xf)) {
					return undef;
				}
				if($islsb) {
					$i |= 0x10;
				};
				return(0x20 | $i);
			} elsif($_[0] eq 'A') {
				if(!defined($ld_arg{$_[1]})) {
					return undef;
				}
				return(0x40 | $ld_arg{$_[1]});
			} elsif($_[1] eq 'A') {
				if(!defined($ld_arg{$_[0]})) {
					return undef;
				}
				return(0x00 | $ld_arg{$_[0]});
			}

			return undef;
		}
	],
);

$op{'JR'} = $op{'JP'};


sub pass {
	my($self, $ifh, $ofh, $passnum) = @_;

	# passnum=0 for plain parsing pass to populate label values
	# passnum=1 for actual pass to assemble

	my($line, $oline, $opcd);

	if($passnum == 0) {
		delete($self->{'label'});
		delete($self->{'binary'});
		delete($self->{'ENTRY'});
		delete($self->{'LUT'});
	}

	seek($ifh, 0, 0); # rewind
	$self->{'pc'} = 0;
	$self->{'line_number'} = 0;
	while(defined($line = <$ifh>)) {
		$self->{'line_number'}++;
		$line =~ s/\s+$//so;
		$oline = $line;
		$line =~ s/;.*//o;
		$line =~ s/^\s+//o;
		@_ = split(/[\s,]+/, $line);

		undef($opcd);

		if(@_ > 0) {

			if(
				($_[0] =~ s/^://o)
				||
				($_[0] =~ s/:$//o)
			) {
				if($passnum == 0) {
					if(defined($self->{'label'}{$_[0]})) {
						die "label redefinition for \"$_[0]\" in line $self->{'line_number'}";
					}
					$self->{'label'}{$_[0]} = $self->{'pc'};
				}
				shift(@_);
			}

			if(@_ > 0) {
				if($passnum == 1) {
					if((@_ == 3) && ($_[1] eq '=')) {
						# convert this = that to LD 
						$_[1] = $_[0];
						$_[0] = 'LD';
					}
					elsif((@_ == 3) && ($_[1] eq '+=')) {
						# convert this += that to ADDER8 or ADDER16
						if($_[0] eq 'A') {
							@_ = ('ADDER8', $_[2]);
						}
						elsif($_[2] eq 'X') {
							@_ = ('ADDER16', $_[0]);
						}
					}
					elsif((@_ == 3) && ($_[1] eq '-=')) {
						# convert this -= that to ADDER8 or ADDER16
						if($_[0] eq 'A') {
							@_ = ('SUB8', $_[2]);
						}
						elsif($_[2] eq 'X') {
							@_ = ('SUB16', $_[0]);
						}
					}
					elsif((@_ == 1) && ($_[0] =~ m/^(B((SET)|(CLR)))([1-4])$/oi)) {
						# convert BSETn or BCLRn to BSET n-1 or BCLR n-1
						$_[0] = $1;
						$_[1] = $5 - 1;
					}

					$op = "\U$_[0]";
					if(!defined($op{$op})) {
						die "unknown instruction: $op in line $self->{'line_number'}";
					}
					shift(@_);

					$op = $op{$op};
					$sub = $op->[1];
					if(defined($sub)) {
						$opcd = &$sub($self, @_);
					} else {
						$opcd = 0;
					}

					if(!defined($opcd)) {
						die "bad argument(s) in line $self->{'line_number'}";
					}

					$opcd |= $op->[0];
				}

				$self->{'pc'}++;
			}

		} else {
			if($passnum == 0) {
				if($oline =~ m/^;LUT; (.*)/o) {
					my($entry, $label) = split(/\s+/, $1);
					$entry =~ s/^0x//o;
					$self->{'LUT'}[hex($entry)] = $label;
				}
				if($oline =~ m/^;ENTRY; (.*)/o) {
					my($id, $label) = split(/\s+/, $1);
					$self->{'ENTRY'}{$id} = $label;
				}
			}
		}

		if($passnum == 1) {
			if(defined($opcd)) {
				$self->{'binary'} .= chr($opcd);

				printf $ofh ("/* 0x%02x */ 0x%02x%s /* %-*s */\n",
					$self->{'pc'} - 1,
					$opcd,
					(
						(
							($self->{'last pc'} < 0xff)
							|| 
							($self->{'last pc'} != $self->{'pc'} - 1)
						) ?
							','
						:
							''
					),
					$self->{'pagewidth'} - 23,
					$oline
				);
			} else {
				if($oline ne '') {
					print $ofh "                 /* $oline */\n";
				} else {
					print $ofh "\n";
				}
			}
		}
	}

	if($passnum == 0) {
		$self->{'last pc'} = $self->{'pc'} - 1;
	}

	if($passnum == 1) {
		while($self->{'pc'} < 0xff) {
			printf $ofh ("/* 0x%02x */ 0,\n",
				$self->{'pc'}
			);
			$self->{'pc'}++;
		}
		if($self->{'pc'} < 0x100) {
			printf $ofh ("/* 0x%02x */ 0\n",
				$self->{'pc'}
			);
			$self->{'pc'}++;
		}
	}
}

} # package DTC_as


use Getopt::Std;

%opt = (
	't' => 'unsigned char',
);

# -t type of arrays (defaults to unsigned char)
# -l lookup table array name (no table generated if not provided)
# -d DTC code array name (naked elements if not provided)
# -i input filename (trailing argument if not provided)
# -o output filename (stdout if not provided)
getopts('l:d:i:o:t:b', \%opt);

if(defined($opt{'i'})) {
	$infile = $opt{'i'};
} else {
	$infile = shift;
}

if(!open(IN, '<', $infile)) {
	die "$infile: $!";
}


if($opt{'b'}) {
	if(!defined($opt{'o'})) {
		die "binary format requires -o";
	}
	if(!open(OUT, '>&', *STDOUT)) {
		die "dup stdout: $!";
	}
	open(STDOUT, '>&', *STDERR);
} else {
	if(defined($opt{'o'})) {
		if(!open(OUT, '>', $opt{'o'})) {
			die "$opt{'o'}: $!";
		}
	} else {
		if(!open(OUT, '>&', *STDOUT)) {
			die "dup stdout: $!";
		}
		open(STDOUT, '>&', *STDERR);
	}
}
	

$as = new DTC_as;

$as->pass(*IN, *OUT, 0);

if(defined($opt{'d'})) {
	print OUT "$opt{'t'} $opt{'d'}", "[0x100] = {\n";
}
$as->pass(*IN, *OUT, 1);
if(defined($opt{'d'})) {
	print OUT "};\n\n";
}

close(IN);

if(defined($opt{'l'})) {
	print OUT "$opt{'t'} $opt{'l'}", "[0x40] = {\n";
#	$end = @{$as->{'LUT'}};
#	if($end > 0x100) {
#		$end = 0x100;
#	}
	for($i = 0xc0; $i < 0x100; $i++) {
		$label = $as->{'LUT'}[$i];
		if(defined($label)) {
			if(defined($as->{'label'}{$label})) {
				printf OUT ("/* 0x%02x */ 0x%02x%s /* %s */\n",
					$i,
					$as->{'label'}{$label},
					(($i < 0xff) ? ',' : ''),
					$label
				);
			} else {
				die "label $label has not been defined";
			}
		} else {
			printf OUT ("/* 0x%02x */ 0%s\n",
				$i,
				(($i < 0xff) ? ',' : ''),
			);
		}
	}
	print OUT "};\n\n";
}


close(OUT);

sub DTCLOAD_COMMENT { 0; }
sub DTCLOAD_ENTRY { 1; }
sub DTCLOAD_LOAD { 2; }
sub DTCLOAD_RUN { 3; }
sub DTCLOAD_LUT_START { 4; }
sub DTCLOAD_LUT { 5; }


if($opt{'b'}) {
	open(OUT, ">", $opt{'o'}) || die "$opt{'o'}: $!";
	syswrite(OUT, pack('CC', DTCLOAD_LUT_COMMENT, 3 - 1) . 'DTC');
	
	$ref = $as->{'LUT'};
	if(@$ref > 0) {
		for($start = 0; $start < @$ref && !defined($ref->[$start]); $start++) {}
		for($end = 0xff; $end >= $start && !defined($ref->[$end]); $end--) {}
		undef($lut);
		for($i = $start; $i <= $end; $i++) {
			if(!defined($ref->[$i])) {
				$lut .= "\0";
				next;
			}
			$label = $ref->[$i];
			if(defined($as->{'label'}{$label})) {
				$label = $as->{'label'}{$label};
#				printf("adding LUT entry 0x%02x\n", $label);
				$lut .= chr($label);
			} else {
				die "label $label has not been defined";
			}
		}
		if(length($lut) > 0) {
			syswrite(OUT, pack('CCC', DTCLOAD_LUT_START, 1 - 1, $start));
			syswrite(OUT, pack('CC', DTCLOAD_LUT, length($lut) - 1) . $lut);
		}
	}

	while(($key, $label) = each(%{$as->{'ENTRY'}})) {
#		print "$key = $label\n";
		if(defined($as->{'label'}{$label})) {
			$label = $as->{'label'}{$label};
#			print "key=$key\n";
#			print "label=$label\n";
			syswrite(OUT, pack('CCC', DTCLOAD_ENTRY, length($key), $label) . $key);
		} else {
			die "label $label has not been defined";
		}
	}

	if(length($as->{'binary'})) {
#		printf("DTC code size: 0x%x\n", length($as->{'binary'}));
		syswrite(OUT, pack('CC',
			DTCLOAD_LOAD ,
			length($as->{'binary'}) - 1
		) . $as->{'binary'});

		if(%{$as->{'ENTRY'}} < 1) {
			syswrite(OUT, pack('CCC', DTCLOAD_RUN, 1 - 1, 0x00));
		}
	}

	close(OUT);
}


0;
