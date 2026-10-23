#!/usr/bin/perl
# cells.pl RUN [TAG]: what the lock-in saw in each eye at a recognition (the driver's cells dump, Lockin.Dump_Cells),
# against the robot's truth: for each regressor (a group's channel) of each eye, its group's true name and role, how
# many textured cells respond, how many of them a whole view would have, how far their motion is one camera motion
# (an affine field fitted to their displacements: the share of their motion it explains), how much their directions
# agree, and with which other regressors' cells they overlap most. TAG: the dump's beat (the last one when not given).
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
my %gname = map { $_->{wire} => "$_->{name}($_->{role})" } @{$truth->{groups}};
my %ename = map { $_->{wire} => "$_->{name}($_->{mount})" } @{$truth->{cameras}};
open(my $d, "<", $file) or die;
my ($eye, %grid, @tex, %col, %cells, $cur);
my @eyes;
while (<$d>) {
  if (/^eye (\d+) width (\d+) height (\d+) columns (\d+) rows (\d+)/) { $eye = $1; push @eyes, $eye; $grid{$eye} = [$2, $3, $4, $5]; next; }
  if (/^textured (\S*)/) { $tex[$eye] = [split //, $1]; next; }
  if (/^column (\d+) group (\d+) channel (\d+)/) { $cur = $1; $col{$eye}{$cur} = [$2, $3]; $cells{$eye}{$cur} = {}; next; }
  if (/^cell (\d+) (\S+) (\S+)/) { $cells{$eye}{$cur}{$1} = [$2, $3]; next; }
}
close $d;
print "dump $file\n";
for my $e (@eyes) {
  my ($w, $h, $nc, $nr) = @{$grid{$e}};
  my $ntex = grep { $_ } @{$tex[$e] || []};
  print "== eye $e ", ($ename{$ekey{$e} // ""} // "?"), ": $ntex textured of ", $nc * $nr, "\n";
  for my $k (sort { $a <=> $b } keys %{$col{$e}}) {
    my ($g, $ch) = @{$col{$e}{$k}};
    my $cs = $cells{$e}{$k}; my @ids = keys %$cs; my $n = @ids;
    next unless $n;
    # cell centres in pixels relative to the picture's middle
    my (@x, @y, @u, @v);
    for my $c (@ids) {
      my ($cx, $cy) = ($c % $nc, int($c / $nc));
      push @x, ($cx + 0.5) * $w / $nc - $w / 2; push @y, ($cy + 0.5) * $h / $nr - $h / 2;
      push @u, $cs->{$c}[0]; push @v, $cs->{$c}[1];
    }
    # direction agreement: |sum of motions| / sum of |motions|
    my ($su, $sv, $sm) = (0, 0, 0);
    for my $i (0 .. $n - 1) { $su += $u[$i]; $sv += $v[$i]; $sm += sqrt($u[$i] ** 2 + $v[$i] ** 2); }
    my $agree = $sm > 0 ? sqrt($su ** 2 + $sv ** 2) / $sm : 0;
    # affine field u = a + b x + c y, v = d + e x + f y by least squares; share of the motion's energy explained
    my $expl = affine(\@x, \@y, \@u, \@v);
    # overlaps with the other columns of this eye
    my @ov;
    for my $l (keys %{$col{$e}}) {
      next if $l == $k; my $o = grep { exists $cells{$e}{$l}{$_} } @ids; next unless $o;
      push @ov, [$l, $o / $n];
    }
    @ov = sort { $b->[1] <=> $a->[1] } @ov;
    my $top = join(" ", map { my ($l, $r) = @$_; my ($gg, $cc) = @{$col{$e}{$l}}; sprintf("%s.c%d:%.2f", $gname{$gkey{$gg} // ""} // "g$gg", $cc, $r) } @ov[0 .. ($#ov < 2 ? $#ov : 2)]);
    printf "  %-24s c%-2d %3d of %3d respond (%.2f), affine explains %.2f, directions agree %.2f | overlaps %s\n",
      ($gname{$gkey{$g} // ""} // "g$g"), $ch, $n, $ntex, $ntex ? $n / $ntex : 0, $expl, $agree, $top;
  }
}
sub affine {
  my ($x, $y, $u, $v) = @_; my $n = @$x; return 0 if $n < 4;
  my $fit = sub {
    my ($z) = @_;   # least squares of z on (1, x, y)
    my @A = ([0,0,0],[0,0,0],[0,0,0]); my @b = (0,0,0);
    for my $i (0 .. $n - 1) { my @r = (1, $x->[$i], $y->[$i]); for my $p (0..2) { $b[$p] += $r[$p] * $z->[$i]; for my $q (0..2) { $A[$p][$q] += $r[$p] * $r[$q]; } } }
    my @s = solve3(\@A, \@b); my $res = 0; my $tot = 0;
    for my $i (0 .. $n - 1) { my $p = $s[0] + $s[1] * $x->[$i] + $s[2] * $y->[$i]; $res += ($z->[$i] - $p) ** 2; $tot += $z->[$i] ** 2; }
    return ($res, $tot);
  };
  my ($ru, $tu) = $fit->($u); my ($rv, $tv) = $fit->($v);
  my $tot = $tu + $tv; return 0 unless $tot > 0;
  return 1 - ($ru + $rv) / $tot;
}
sub solve3 {
  my ($A, $b) = @_; my @M = map { [@{$A->[$_]}, $b->[$_]] } 0..2;
  for my $c (0..2) {
    my $p = $c; for my $r ($c+1..2) { $p = $r if abs($M[$r][$c]) > abs($M[$p][$c]); }
    @M[$c, $p] = @M[$p, $c]; return (0,0,0) if abs($M[$c][$c]) < 1e-12;
    for my $r (0..2) { next if $r == $c; my $f = $M[$r][$c] / $M[$c][$c]; $M[$r][$_] -= $f * $M[$c][$_] for $c..3; }
  }
  return map { $M[$_][3] / $M[$_][$_] } 0..2;
}
