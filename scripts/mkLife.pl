#!/usr/bin/perl

use warnings;

die "\nUsage: $0 NAME\n\topens NAME.cells and writes NAME.h\n\n" if @ARGV <1;

($name = $ARGV[0]) =~ s/\.[^.]*$//;

open $TFILE, "$name\.cells" or die "CELLS File not found: $name.cells";
open ($HFILE, '>', "$name\.h") or die "Cound not open BMAP: $name.h";

$row = 384/2;

while ( <$TFILE> ) {
	chomp $_;
	next if ( $_ =~ '^!' );
	@cell = split(//, $_);
	$col = 512/2;
	for ( @cell ) {
		printf $HFILE "setLifeCell(lifeGen, %d, %s, 1);\n", $col, $row if ( $_ eq 'O' );
		$col++;
	}
	$row++;
}

close HFILE;
