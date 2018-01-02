#!/usr/bin/perl
#
# Helper for generating GPIO setup for NXP LPC43 series
#
# Each pin is configured by "PortAndBit:Conf:Speed"
#  'PortAndBit' specifies Port and bit number (e. g. PTB03)
#  'Conf' is one of 'AFx' (alternate), 'P' (output), 'IN' (input),
#    (each optionally by 'P' (push-pull) or 'O' (open-drain)),
#    (all optionally followed by 'UP' (pull-up), or 'DO' (pull-down))
#  'Speed' is one of 'L' (low), 'H' (high)
#
# Port configuration can be given on command line as a single string (pins separated by commas)
#
# The required pin multiplexing (i. e. Px_xx->FUNCy->GPIOz[zz]) is extracted from the chip's
# user manual, the relevant page range for 'Pin multiplexing table' must be specified.
#
# Pins have to be ordered this way:
#  - I2C: SDA, SCL
#  - SPI (1 line): NCS, CLK, IO1/MISO, IO0/MOSI
#  - DPI (2 lines): NCS, CLK, IO1/MISO, IO0/MOSI
#  - QPI (4 lines): NCS, CLK, IO3/NHOLD, IO2/NWP, IO1/MISO, IO0/MOSI
#

use strict;
use Getopt::Std;

my $PIO_BASE;
my $Conf = "P3_8:PPUP:L, P3_3:PPUP:H, P3_4:INPUP:H, P3_5:INPUP:H, P3_6:INPUP:H, P3_7:INPUP:H";
#my $Conf = "P2_03:INPUP:L, P2_4:INPUP:L";

my $GPIO_BASE = 0x400F4000;

&getopts('b:c:p:u:');
if ($Getopt::Std::opt_b eq '')
{
	if ($GPIO_BASE eq '')
	{
		die("usage: $0 -b io_base -p first_page-last_page -u user_manual.pdf [ -c port_configuration ]");
	}
}
else
{
	$GPIO_BASE = eval $Getopt::Std::opt_b;
}

if (($Getopt::Std::opt_p eq '') || ($Getopt::Std::opt_u eq '') ||
	!(-r $Getopt::Std::opt_u))
{
	die("usage: $0 -b io_base -p first_page-last_page -u user_manual.pdf [ -c port_configuration ]");
}

if ($Getopt::Std::opt_c eq '')
{
	if (($Conf eq '') && ($Getopt::Std::opt_f eq ''))
	{
		die("usage: $0 [ -b io_base ] -p first_page-last_page -u user_manual.pdf -c port_configuration");
	}
}
else
{
	$Conf = $Getopt::Std::opt_c . ',';
}

$Getopt::Std::opt_p =~ /^([0-9]+)-([0-9]+)$/;
if (($1 <= 0) || ($2 <= 0) || ($1 > $2))
{
	die("invalid page rage");
}

my $cmd = "pdftotext -layout -f $1 -l $2 '${Getopt::Std::opt_u}' - | "
	. "sed -rn '/^\\s*P[0-9A-F]_[0-9]+/s/^\\s+//p'";
open(PIN_LIST, "${cmd} |") || die();
my $lineno = 0;
my %Pins;
while (my $line = <PIN_LIST>)
{
	my $pin;
	$lineno++;

	if ($line =~ /^(P[0-9A-F]_[0-9]+)/)
	{
		$pin = $1;
		$pin =~ s/_([0-9])$/_0\1/;
		$line =~ s/^\w+\s+//;
		$Pins{$pin} = [ split(/\s+/, $line) ];
	}
	else
	{
		die("line ${lineno}:'$line' invalid");
	}
}
close(PIN_LIST);

my $Sep = "\t";
my $Form = "${Sep}mmw 0x%08X 0x%08X 0x%08X\t;# ";

my $GPIO_OFFS = 4;
my $GPIO_DIR = 0x2000;
my $GPIO_PIN = 0x2100;
my $SCU_BASE = 0x40086000;
my $SFSP_OFFS = 0x80;

my @Out = ( { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { } );
my @GPIO = ( );
my @SCU = ( );
my $Exor;
my %Conf;
my $Pins = "${Sep}#";

my $pins;
my $altn;
my %defs;

{
	my @names = ( );
	my @conf = split(/\s*,\s*/, $Conf);

	if (@conf == 2)
	{
		push(@names, 'SDA', 'SCL');
	}
	else
	{
		push(@names, 'NCS', 'CLK');
		if (@conf == 3)
		{
			push(@names, 'IO0/DIO');
		}
		elsif (@conf == 4)
		{
			push(@names, 'IO1/MISO', 'IO0/MOSI');
		}
		elsif (@conf == 6)
		{
			push(@names, 'IO3/NHOLD', 'IO2/NWP', 'IO1/MISO', 'IO0/MOSI');
		}
		else
		{
			die("invalid config");
		}
	}

	for (my $index = 0; $index < @conf; $index++)
	{
		uc($conf[$index]) =~ /^P([A-K])([0-9]+):\s*([A-Z0-9]+)$/;
		$Pins .= sprintf(" %s: P%s%02d,", $names[$index], $1, $2);
	}
	chop($Pins);
}

my @Col = ( );
my @conf = split(/\s*,\s*/, $Conf);

if (@conf == 3)
{
	splice(@conf, 2, 0, 'NONE', 'NONE', 'NONE');
}
elsif (@conf == 4)
{
	splice(@conf, 2, 0, 'NONE', 'NONE');
}

foreach my $line (@conf)
{
	$line = uc($line);
	$line =~ /^P([0-9A-F])_([0-9]+):\s*([A-Z0-9]+):(L|H)$/;
	my $group = &fromhex($1);
	my $pin = $2;
	my $conf = $3;
	my $speed = $4;
	my $pad = sprintf("P%s_%02d", $1, $pin);
	my $gpio;
	my $bit;
	my $func;
	my $sfsp;

	for ($func = 0; $func < @{$Pins{$pad}}; $func++)
	{
		($func >= 8) && last;
		if (${$Pins{$pad}}[$func] =~ /GPIO([0-9])\[([0-9]+)\]/)
		{
			$gpio = $1;
			$bit = $2;
			last;
		}
	}
	if ($gpio eq '')
	{
		printf(STDERR "invalid pad %s\n", $pad);
		push(@{$GPIO[16]}, sprintf("%s->none:%s:%s", $pad, $conf, $speed));
		push(@Col, sprintf("0x%08X %2d", 0, 0));
		next;
	}

	my $num = ord(${gpio}) - ord('0');
	my $out = $Out[$num];
	my $mask = 0x0;

	if ($conf eq '')
	{
		if ($line ne 'NONE')
		{
			printf(STDERR "invalid conf %s\n", $line);
		}
		next;
	}
	elsif ($conf =~ /^IN(|P|O)(|UP|DO)$/)
	{
		if ($1 eq 'O')
		{
			printf(ERROR "open-drain not available %s\n", $line);
		}
		elsif ($1 eq 'P')
		{
			$mask |= (0x1 << ${bit});
		}
		$$out{'DIR_L'} |= (0x1 << ${bit});
		$$out{'PIN_H'} |= (0x1 << ${bit});
		$sfsp = 0x40 | (($speed eq 'H') ? 0xA0 : 0x00) | $func;
		$sfsp |= ($2 eq '') ? 0x10 : (($2 eq 'DO') ? 0x80 : 0x00);
	}
	elsif ($conf =~ /^P(P|O)(|UP|DO)$/)
	{
		if ($1 eq 'O')
		{
			printf(ERROR "open-drain not available %s\n", $line);
		}
		$$out{'DIR_H'} |= (0x1 << ${bit});
		$$out{'PIN_H'} |= (0x1 << ${bit});
		$sfsp = 0x40 | (($speed eq 'H') ? 0xA0 : 0x00) | $func;
		$sfsp |= ($2 eq '') ? 0x10 : (($2 eq 'DO') ? 0x80 : 0x00);
	}
	else
	{
		printf(STDERR "invalid conf %s\n", $line);
		next;
	}

	my $Exor = sprintf("0x%08X %2d", ${GPIO_BASE} + $num * ${GPIO_OFFS} + ${GPIO_PIN}, $bit);
	($mask & (0x1 << ${bit})) && ($Exor .= sprintf(" 0x%03X 0x%03X 0x%08X",
		(${GPIO_DIR}-${GPIO_PIN}) & 0x3FF, (${GPIO_DIR}-${GPIO_PIN}) & 0x3FF, $mask));

	push(@{$GPIO[$num]}, sprintf("%s->FUNC%0d->GPIO%s[%02d]:%s:%s", $pad, $func, $gpio, $bit, $conf, $speed));
	push(@{$SCU[$num]}, sprintf("\tmww 0x%08X 0x%02X\t\t\t\t\t\t;# SFSP%d_%02d\n",
		$SCU_BASE + $group * $SFSP_OFFS + 4 * $pin, $sfsp, $group, $pin));
	push(@Col, $Exor);
}

my $Col = sprintf("${Sep}0x%03X ", (${GPIO_PIN}-${GPIO_PIN}) & 0x3FF);
for (my $i = 0; $i < @Col; $i++)
{
	if (($i != 0) && (($i % 2) == 0))
	{
		(($i + 1) < @Col) && ($Col .= "\\\n${Sep}");
	}
	$Col .= sprintf("%s ", $Col[$i]);
}
printf("\n%s\n", $Col);

my @Col = ( );
my $Set;
for (my $i = 0; $i < @Out; $i++)
{
	my $out = $Out[$i];
	my $addr = ${GPIO_BASE} + $i * ${GPIO_OFFS};
	my $count = 0;

	if ($i < 16)
	{
		if (($$out{'DIR_H'} | $$out{'DIR_L'} |
			$$out{'PIN_H'} | $$out{'PIN_L'}) != 0)
		{
			push(@Col, sort({ $a cmp $b } @{$GPIO[$i]}));

			$Set .= sprintf("%s# GPIO%s: %s\n", ${Sep}, tohex($i),
				join(", ", sort({ $b cmp $a } @{$GPIO[$i]})));

			$Set .= join("", sort({ $a cmp $b } @{$SCU[$i]}));

			(($$out{'DIR_H'} != 0) || ($$out{'DIR_L'} != 0)) &&
				($Set .= sprintf("${Form}GPIO_DIR%d\n", $addr + ${GPIO_DIR},
					$$out{'DIR_H'}, $$out{'DIR_L'}, $i));

			(($$out{'PIN_H'} != 0) || ($$out{'PIN_L'} != 0)) &&
				($Set .= sprintf("${Form}GPIO_PIN%d\n", $addr + ${GPIO_PIN},
					$$out{'PIN_H'}, $$out{'PIN_L'}, $i));
		}
	}
	elsif (ref $GPIO[$i] eq 'ARRAY')
	{
		$Set .= sprintf("%s# No GPIO: %s\n", ${Sep},
			join(", ", sort({ $b cmp $a } @{$GPIO[$i]})));
	}
}

my $Col = '';
for (my $i = 0; $i < @Col; $i++)
{
	if (($i % 6) == 0)
	{
		chop($Col);
		(($i + 1) < @Col) && ($Col .= "\n${Sep}#");
	}
	$Col .= sprintf(" %s,", $Col[$i]);
}
chop($Col);
printf("%s\n", $Col);
printf("%s\n", $Set);

sub fromhex($)
{
	my $val;

	$val = ord($_[0]) - ord('0');
	return (($val < 10) ? $val : $val - ord('A') - ord('0'));
}

sub tohex($)
{
	return (($_[0] < 10) ? chr($_[0] + ord('0')) : chr($_[0] - 10 + ord('A')));
}
