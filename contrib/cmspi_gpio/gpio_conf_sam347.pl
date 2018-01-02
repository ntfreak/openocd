#!/usr/bin/perl
#
# Helper for generating GPIO setup for Atmel/Microchip SAM3, SAM4, SAM7 series
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
# Pins have to be ordered this way:
#  - I2C: SDA, SCL
#  - SPI (1 line): NCS, CLK, IO1/MISO, IO0/MOSI
#  - DPI (2 lines): NCS, CLK, IO1/MISO, IO0/MOSI
#  - QPI (4 lines): NCS, CLK, IO3/NHOLD, IO2/NWP, IO1/MISO, IO0/MOSI
#

use strict;
use Getopt::Std;

my $PIO_BASE;
my $Conf;

$PIO_BASE = 0x400E0E00;

# arduino-due quad-a/b
#$Conf = "PA16:PPUP:L, PA23:PPUP:H, PC28:INPUP:H, PB25:INPUP:H, PA06:INPUP:H, PA22:INPUP:H";
#$Conf = "PA24:PPUP:L, PA23:PPUP:H, PC23:INPUP:H, PC24:INPUP:H, PC25:INPUP:H, PC26:INPUP:H";

&getopts('b:c:');
if ($Getopt::Std::opt_b eq '')
{
	if ($PIO_BASE eq '')
	{
		die("usage: $0 [ -1 ] -b io_base [ -c port_configuration ]");
	}
}
else
{
	$PIO_BASE = eval $Getopt::Std::opt_b;
}

if ($Getopt::Std::opt_c eq '')
{
	if (($Conf eq '') && ($Getopt::Std::opt_f eq ''))
	{
		die("usage: $0 [ -b io_base ] -c port_configuration");
	}
}
else
{
	$Conf = $Getopt::Std::opt_c . ',';
}

my $Sep = "\t";
my $Form = "${Sep}mww 0x%08X 0x%08X\t;# ";

my $PIO_OFFS = 0x200;
my $PIO_PER = 0x00;
my $PIO_PDR = 0x04;
my $PIO_OER = 0x10;
my $PIO_ODR = 0x14;
my $PIO_SODR = 0x30;
my $PIO_CODR = 0x34;
my $PIO_ODSR = 0x38;
my $PIO_PDSR = 0x3C;
my $PIO_MDER = 0x50;
my $PIO_MDDR = 0x54;
my $PIO_PUDR = 0x60;
my $PIO_PUER = 0x64;
my $PIO_OWER = 0xA0;
my $PIO_OWDR = 0xA4;

my @Out = ( { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { } );
my @Port = ( );
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
	$line =~ /^P([A-K])([0-9]+):\s*([A-Z0-9]+):(L|H)$/;
	my $port = $1;
	my $pin = $2;
	my $conf = $3;
	my $speed = $4;

	my $num = ord(${port}) - ord('A');
	my $out = $Out[$num];
	my $mask = 0x0;
	my $AFR = 0x0;

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
		$$out{'PER'} |= (0x1 << ${pin});
		$$out{'OWER'} |= (0x1 << ${pin});
		($1 eq 'O') ? ($$out{'MDER'} |= (0x1 << ${pin})) : ($$out{'MDDR'} |= (0x1 << ${pin}));
		if ($2 eq 'DO')
		{
			printf(STDERR "pull-down not available %s\n", $line);
		}
		($2 eq 'UP') ? ($$out{'PUER'} |= (0x1 << ${pin})) : ($$out{'PUDR'} |= (0x1 << ${pin}));
		($2 eq 'UP') ? ($$out{'SODR'} |= (0x1 << ${pin})) : 0;
		$$out{'ODR'} |= (0x1 << ${pin});
		$mask |= (0x1 << ${pin});
	}
	elsif ($conf =~ /^P(P|O)(|UP|DO)$/)
	{
		$$out{'PER'} |= (0x1 << ${pin});
		$$out{'OWER'} |= (0x1 << ${pin});
		($1 eq 'O') ? ($$out{'MDER'} |= (0x1 << ${pin})) : ($$out{'MDDR'} |= (0x1 << ${pin}));
		if ($2 eq 'DO')
		{
			printf(STDERR "pull-down not available %s\n", $line);
			$$out{'CODR'} |= (0x1 << ${pin});
		}
		($2 eq 'UP') ? ($$out{'PUER'} |= (0x1 << ${pin})) : ($$out{'PUDR'} |= (0x1 << ${pin}));
		($2 eq 'UP') ? ($$out{'SODR'} |= (0x1 << ${pin})) : 0;
		$$out{'OER'} |= (0x1 << ${pin});
	}
	else
	{
		printf(STDERR "invalid conf %s\n", $line);
		next;
	}

	my $Exor = sprintf("0x%08X %2d", ${PIO_BASE} + (ord($port)  - ord('A')) * ${PIO_OFFS} + ${PIO_ODSR}, $pin);
	($mask & (0x1 << ${pin})) && ($Exor .= sprintf(" 0x%03X 0x%03X 0x%08X",
		(${PIO_ODR}-${PIO_ODSR}) & 0x3FF, (${PIO_OER}-${PIO_ODSR}) & 0x3FF, $mask));

	push(@{$Port[$num]}, sprintf("P%s%02d:%s:%s", $port, $pin, $conf, $speed));
	push(@Col, $Exor);
}

my $Col = sprintf("${Sep}0x%03X ", (${PIO_PDSR}-${PIO_ODSR}) & 0x3FF);
for (my $i = 0; $i < @Col; $i++)
{
	if (($i != 0) && (($i % 2) == 0))
	{
		(($i + 1) < @Col) && ($Col .= "\\\n${Sep}");
	}
	$Col .= sprintf("%s ", $Col[$i]);
}
printf("%s\n", $Col);

my @Col = ( );
my $Set;
for (my $i = 0; $i < @Out; $i++)
{
	my $out = $Out[$i];
	my $addr = ${PIO_BASE} + $i * ${PIO_OFFS};
	my $count = 0;

	{
		if (($$out{'PER'} | $$out{'OER'} | $$out{'ODR'} |
			$$out{'SODR'} | $$out{'CODR'} | $$out{'ODSR'} |
			$$out{'MDER'} | $$out{'MDDR'} |
			$$out{'PUDR'} | $$out{'PUER'} |
			$$out{'OWER'} | $$out{'OWDR'}) != 0)
		{
			push(@Col, sort({ $b cmp $a } @{$Port[$i]}));

			$Set .= sprintf("%s# Port %s: %s\n", ${Sep}, chr($i + ord('A')),
				join(", ", sort({ $b cmp $a } @{$Port[$i]})));

			($$out{'PER'} != 0) &&
				($Set .= sprintf("${Form}PIO_PER\n", $addr + ${PIO_PER}, $$out{'PER'}));

			($$out{'OWER'} != 0) &&
				($Set .= sprintf("${Form}PIO_OWER\n", $addr + ${PIO_OWER}, $$out{'OWER'}));
			($$out{'OWDR'} != 0) &&
				($Set .= sprintf("${Form}PIO_OWDR\n", $addr + ${PIO_OWDR}, $$out{'OWDR'}));

			($$out{'OER'} != 0) &&
				($Set .= sprintf("${Form}PIO_OER\n", $addr + ${PIO_OER}, $$out{'OER'}));
			#($$out{'ODR'} != 0) &&
			#	($Set .= sprintf("${Form}PIO_ODR\n", $addr + ${PIO_ODR}, $$out{'ODR'}));

			($$out{'SODR'} != 0) &&
				($Set .= sprintf("${Form}PIO_SODR\n", $addr + ${PIO_SODR}, $$out{'SODR'}));
			($$out{'CODR'} != 0) &&
				($Set .= sprintf("${Form}PIO_CODR\n", $addr + ${PIO_CODR}, $$out{'CODR'}));

			($$out{'MDER'} != 0) &&
				($Set .= sprintf("${Form}PIO_MDER\n", $addr + ${PIO_MDER}, $$out{'MDER'}));
			#($$out{'MDDR'} != 0) &&
			#	($Set .= sprintf("${Form}PIO_MDDR\n", $addr + ${PIO_MDDR}, $$out{'MDDR'}));

			#($$out{'PUDR'} != 0) &&
			#	($Set .= sprintf("${Form}PIO_PUDR\n", $addr + ${PIO_PUDR}, $$out{'PUDR'}));
			($$out{'PUER'} != 0) &&
				($Set .= sprintf("${Form}PIO_PUER\n", $addr + ${PIO_PUER}, $$out{'PUER'}));
		}
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
#printf("\n%s\n", $Pins);
printf("%s\n", $Col);
printf("%s\n", $Set);
