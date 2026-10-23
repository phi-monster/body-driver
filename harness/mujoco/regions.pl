#!/usr/bin/perl
# regions.pl RUN: simulator-side rig code. From the driver's last cells dump of a run (Lockin.Dump_Cells: each group's
# responding cells by the lock-in's test of its block): for each eye, beside the truth (its mount, the group that carries
# it), each group's share of the eye's textured cells and of its textured border cells that respond, and every cell's set
# of responding groups (which groups move it), counted by set. For studying what tells the world in an eye's view.
use strict; use warnings; use JSON::PP;
my $R = shift or die "regions.pl RUN\n";
my @dumps = sort { ($a =~ /(\d+)\.txt$/)[0] <=> ($b =~ /(\d+)\.txt$/)[0] } glob("$R/cells_*.txt");
exit unless @dumps;
open(my $t, "<", "$R/truth.json") or die; my $truth = do { local $/; decode_json(<$t>) }; close $t;
my $L = -e "$R/driver.log" ? "$R/driver.log" : "$R/driver.log.zst";
open(my $f, (-e "$R/driver.log" ? "<" : "-|"), (-e "$R/driver.log" ? $L : "zstd -dc $L")) or die;
my (%cam_key, %group_key, $in);
while (my $line = <$f>) {
  if ($line =~ /\[core\] the robot reports:/) { $in = 1; next; }
  if ($in) {
    if ($line =~ /^camera (\d+): [^\/\s]+\/([^\/\s]+)\//) { $cam_key{$1} = $2; next; }
    if ($line =~ /^group (\d+): (?:[^\/\s]+\/)*([^\/\s,]+),/) { $group_key{$1} = $2; next; }
    last if $line =~ /^\s*$/;
  }
}
close $f;
my %group_by_key = map { $_->{wire} => $_ } @{$truth->{groups}};
my %cam_by_key = map { $_->{wire} => $_ } @{$truth->{cameras}};
my ($name) = $R =~ m#([^/]+)/?$#;
open(my $d, "<", $dumps[-1]) or die;
my ($eye, $cols, $rows, @tex, %resp);
sub gname { my $g = $group_by_key{$group_key{$_[0]} // ""}; return $g ? $g->{name} : "g$_[0]"; }
sub flush {
  return unless defined $eye;
  my $tc = $cam_by_key{$cam_key{$eye} // ""} or return;
  my @cells = grep { $tex[$_] } 0 .. $#tex;
  my @border = grep { my $x = $_ % $cols; my $y = int($_ / $cols); $x == 0 || $y == 0 || $x == $cols - 1 || $y == $rows - 1 } @cells;
  printf "%-10s eye %-18s %-16s on %-12s textured %3d (border %2d)\n", $name, $tc->{name}, $tc->{mount}, $tc->{by} // "-",
    scalar(@cells), scalar(@border);
  for my $g (sort { $a <=> $b } keys %resp) {
    my $n = grep { substr($resp{$g}, $_, 1) eq "1" } @cells;
    my $nb = grep { substr($resp{$g}, $_, 1) eq "1" } @border;
    my $tg = $group_by_key{$group_key{$g} // ""};
    printf "    group %-12s %-8s %-7s respond %3d (%.2f), border %2d (%.2f)\n", gname($g), $tg ? $tg->{role} : "?",
      (defined $tc->{by} && $tg && $tc->{by} eq $tg->{name}) ? "CARRIES" : "-", $n, @cells ? $n / @cells : 0, $nb,
      @border ? $nb / @border : 0;
  }
  my %sets;
  for my $c (@cells) {
    my @s = map { gname($_) } grep { substr($resp{$_}, $c, 1) eq "1" } sort { $a <=> $b } keys %resp;
    $sets{@s ? join("+", @s) : "none"}++;
  }
  printf "    cells by the groups moving them: %s\n", join(", ", map { "$_ $sets{$_}" } sort { $sets{$b} <=> $sets{$a} } keys %sets);
}
while (<$d>) {
  if (/^eye (\d+) width \d+ height \d+ columns (\d+) rows (\d+)/) { flush(); ($eye, $cols, $rows) = ($1, $2, $3); %resp = (); @tex = (); next; }
  if (/^textured (\S+)/) { @tex = split //, $1; next; }
  if (/^responds (\d+) (\S+)/) { $resp{$1} = $2; next; }
}
flush();
