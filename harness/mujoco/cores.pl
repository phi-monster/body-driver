#!/usr/bin/perl
# cores.pl RUN [TAG]: from the driver's cells dump (Lockin.Dump_Cells, each regressor's cells that follow that channel
# alone), for each eye: each group's cells (those that follow at least half of its channels), how many of the eye's
# textured cells they are, and for each pair of groups how many of one's cells are also the other's: the table that
# tells a group that moves the eye (its cells: the world, and every robot part not riding on the eye) from one riding on
# the eye (its cells follow nothing that moves the eye) and one elsewhere (its cells follow the eye's movers too).
use strict; use warnings; use JSON::PP;
my ($R, $tag) = @ARGV;
my @dumps = sort { ($a =~ /(\d+)\.txt$/)[0] <=> ($b =~ /(\d+)\.txt$/)[0] } glob("$R/cells_*.txt");
die "no dumps in $R\n" unless @dumps;
my $file = defined $tag ? "$R/cells_$tag.txt" : $dumps[-1];
open(my $t, "<", "$R/truth.json") or die; my $truth = do { local $/; decode_json(<$t>) }; close $t;
my $L = -e "$R/driver.log" ? "$R/driver.log" : "$R/driver.log.zst";
open(my $f, (-e "$R/driver.log" ? "<" : "-|"), (-e "$R/driver.log" ? $L : "zstd -dc $L")) or die;
my (%gkey, %ekey, $in);
while (<$f>) {
  if (/\[core\] the robot reports:/) { $in = 1; next; }
  if ($in) { $gkey{$1} = $2 if /^group (\d+): (?:[^\/\s]+\/)*([^\/\s,]+),/; $ekey{$1} = $2 if /^camera (\d+): [^\/\s]+\/([^\/\s]+)\//; last if /^\s*$/ && %gkey; }
}
close $f;
my %gname = map { $_->{wire} => "$_->{name}:$_->{role}" } @{$truth->{groups}};
my %ename = map { $_->{wire} => "$_->{name}:$_->{mount}" } @{$truth->{cameras}};
open(my $d, "<", $file) or die;
my ($eye, @eyes, %ntex, %colgroup, %resp, $cur);
while (<$d>) {
  if (/^eye (\d+)/) { $eye = $1; push @eyes, $eye; next; }
  if (/^textured (\S*)/) { $ntex{$eye} = () = $1 =~ /1/g; next; }
  if (/^column (\d+) group (\d+) channel (\d+)/) { $cur = $1; $colgroup{$eye}{$cur} = $2; next; }
  if (/^cell (\d+) /) { $resp{$eye}{$cur}{$1} = 1; next; }
}
close $d;
print "dump $file\n";
for my $e (@eyes) {
  my %cols; push @{$cols{$colgroup{$e}{$_}}}, $_ for keys %{$colgroup{$e}};
  my %cells;   # group -> cells following at least half its channels
  for my $g (keys %cols) {
    my %n; for my $k (@{$cols{$g}}) { $n{$_}++ for keys %{$resp{$e}{$k} || {}}; }
    my $need = @{$cols{$g}} / 2;
    $cells{$g} = { map { $_ => 1 } grep { $n{$_} >= $need } keys %n };
  }
  my @g = sort { scalar(keys %{$cells{$b}}) <=> scalar(keys %{$cells{$a}}) } grep { %{$cells{$_}} } keys %cells;
  print "== eye $e ", ($ename{$ekey{$e} // ""} // "?"), ", $ntex{$e} textured\n";
  for my $a (@g) {
    my $na = keys %{$cells{$a}};
    my @row = map { my $b = $_; my $o = grep { $cells{$b}{$_} } keys %{$cells{$a}}; sprintf("%s %.2f", (split /:/, $gname{$gkey{$b} // ""} // "g$b")[0], $o / $na) } grep { $_ != $a } @g;
    printf "  %-22s %2d ch, %3d cells (%.2f) | of them also: %s\n", ($gname{$gkey{$a} // ""} // "g$a"), scalar(@{$cols{$a}}), $na, $na / ($ntex{$e} || 1), join(", ", @row);
  }
}
