#!/usr/bin/perl
#
# Helper for generating GPIO setup for Kinetis Kx series
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

my $GPIO_BASE;
my $PCR_BASE;
my $Conf;

$GPIO_BASE = 0x400FF000;
$PCR_BASE = 0x40049000;

&getopts('b:c:');
if ($Getopt::Std::opt_b eq '')
{
	if ($GPIO_BASE eq '')
	{
		die("usage: $0 [ -1 ] -b io_base [ -c port_configuration ]");
	}
}
else
{
	$GPIO_BASE = eval $Getopt::Std::opt_b;
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
my $Form = "${Sep}mmw 0x%08X 0x%08X 0x%08X\t;# ";

my $GPIO_OFFS = 0x40;
my $GPIO_PODR = 0x00;
my $GPIO_PIDR = 0x10;
my $GPIO_PDDR = 0x14;
my $PCR_OFFS = 0x1000;

my @Out = ( { }, { }, { }, { }, { }, { }, { }, { }, { }, { }, { } );
my @Port = ( );
my @PCR = ( );
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
		uc($conf[$index]) =~ /^PT?([A-K])([0-9]+):\s*([A-Z0-9]+)$/;
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
	$line =~ /^PT?([A-K])([0-9]+):\s*([A-Z0-9]+):(L|H)$/;
	my $port = $1;
	my $pin = $2;
	my $conf = $3;
	my $speed = $4;

	my $num = ord(${port}) - ord('A');
	my $out = $Out[$num];
	my $exor = 0x0;
	my $PCR_H = ($speed eq 'L') ? (0x1 << 2) : 0x0;
	my $PCR_L = ($speed eq 'H') ? (0x1 << 2) : 0x0;
	my $AFR = 0x0;

	if ($conf eq '')
	{
		if ($line ne 'NONE')
		{
			printf(STDERR "invalid conf %s\n", $line);
		}
		next;
	}
	elsif ($conf =~ /^AF([0-9]+)(|P|O)(|UP|DO)$/)
	{
		if (($1 < 0) || ($1 == 1) || ($1 > 7))
		{
			printf(STDERR "invalid alternate %s\n", $line);
			next;
		}
		$AFR = $1;
		if ($2 ne '')
		{
			($2 eq 'O') ? ($PCR_H |= (0x1 << 5)) : ($PCR_L |= (0x1 << 5));
		}
		$PCR_H |= ($AFR << 8);
		$PCR_L |= (($AFR ^ 0x7) << 8);
		if ($3 eq '')
		{
			$PCR_L |= (0x1 << 1);
		}
		else
		{
			$PCR_H |= (0x1 << 1);
			($3 eq 'UP') ? ($PCR_H |= (0x1 << 0)) : ($PCR_L |= (0x1 << 0));
		}
		$conf = sprintf("AF%02d%s%s", $AFR, $3, $4);
	}
	elsif ($conf =~ /^IN(|P|O)(|UP|DO)$/)
	{
		if ($1 ne '')
		{
			($1 eq 'O') ? ($PCR_H |= (0x1 << 5)) : ($PCR_L |= (0x1 << 5));
		}
		if ($2 eq '')
		{
			$PCR_L |= (0x1 << 1);
		}
		else
		{
			$PCR_H |= (0x1 << 1);
			($2 eq 'UP') ? ($PCR_H |= (0x1 << 0)) : ($PCR_L |= (0x1 << 0));
		}
		$$out{'PDDR_L'} |= (0x1 << ${pin});
		$$out{'PODR_H'} |= (0x1 << ${pin});
		$PCR_H |= (0x1 << 8);
		$PCR_L |= (0x6 << 8);
		$exor |= (0x1 << ${pin});
	}
	elsif ($conf =~ /^P(P|O)(|UP|DO)$/)
	{
		($1 eq 'O') ? ($PCR_H |= (0x1 << 5)) : ($PCR_L |= (0x1 << 5));
		if ($2 eq '')
		{
			$PCR_L |= (0x1 << 1);
		}
		else
		{
			$PCR_H |= (0x1 << 1);
			($2 eq 'UP') ? ($PCR_H |= (0x1 << 0)) : ($PCR_L |= (0x1 << 0));
		}
		$PCR_H |= (0x1 << 8);
		$PCR_L |= (0x6 << 8);
		$$out{'PDDR_H'} |= (0x1 << ${pin});
		$$out{'PODR_H'} |= (0x1 << ${pin});
		#$exor |= (0x1 << ${pin});
	}
	else
	{
		printf(STDERR "invalid conf %s\n", $line);
		next;
	}

	push(@{$PCR[$num]}, sprintf("${Sep}mmw 0x%08X 0x%08X 0x%08X${Sep};# PORT%s_PCR%02d\n",
		${PCR_BASE} + (ord($port)  - ord('A')) * ${PCR_OFFS} + (${pin} << 2), $PCR_H, $PCR_L, $port, $pin));
	my $Exor = sprintf("0x%08X %2d", ${GPIO_BASE} + (ord($port)  - ord('A')) * ${GPIO_OFFS} + ${GPIO_PODR}, $pin);
	($exor & (0x1 << ${pin})) && ($Exor .= sprintf(" 0x%03X 0x%03X 0x%08X", (${GPIO_PDDR}-${GPIO_PODR}) & 0x3FF,
		(${GPIO_PDDR}-${GPIO_PODR}) & 0x3FF, $exor));

	push(@{$Port[$num]}, sprintf("PT%s%02d:%s:%s", $port, $pin, $conf, $speed));
	push(@Col, $Exor);
}

my $Col = sprintf("${Sep}0x%03X ", (${GPIO_PIDR}-${GPIO_PODR}) & 0x3FF);
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
	my $addr = ${GPIO_BASE} + $i * ${GPIO_OFFS};
	my $count = 0;

	if (($$out{'PDDR_H'} | $$out{'PDDR_L'}) != 0)
	{
		push(@Col, sort({ $b cmp $a } @{$Port[$i]}));

		$Set .= sprintf("%s# Port %s: %s\n", ${Sep}, chr($i + ord('A')),
			join(", ", sort({ $b cmp $a } @{$Port[$i]})));
		$Set .= join("", sort({ $a cmp $b } @{$PCR[$i]}));

		(($$out{'PODR_H'} | $$out{'PODR_L'}) != 0) &&
			($Set .= sprintf("${Form}GPIO%s_PODR\n", $addr + ${GPIO_PODR},
				$$out{'PODR_H'}, $$out{'PODR_L'}, chr($i + ord('A'))));
		(($$out{'PDDR_H'} | $$out{'PDDR_L'}) != 0) &&
			($Set .= sprintf("${Form}GPIO%s_PDDR\n", $addr + ${GPIO_PDDR},
				$$out{'PDDR_H'}, $$out{'PDDR_L'}, chr($i + ord('A'))));
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
