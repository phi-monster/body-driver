#!/usr/bin/perl
# step2both.pl RUNS: simulator-side rig code. Step 2's two conditions on every arm a boot swept: its focal lengths
# within 2 % at its last verdict, fitted (focal3.pl), and its order right and told by a fit that held (order3.pl).
use strict; use warnings; use FindBin;
my $runs = shift or die "step2both.pl RUNS\n";
my (%foc, %ord, %n, %b);
open(my $f, "-|", "perl $FindBin::Bin/focal3.pl $runs 2>/dev/null") or die;
while (<$f>) { next unless /^(\S+)\s+arm (\d+) /; my $k = "$1 $2"; $foc{$k} = /within 2%/ ? 1 : 0 }
close $f;
open(my $o, "-|", "perl $FindBin::Bin/order3.pl $runs 2>/dev/null") or die;
while (<$o>) { next unless /^(\S+)\s+arm (\d+) /; my $k = "$1 $2"; $ord{$k} = /RIGHT told$/ ? 1 : 0 }
close $o;
for my $k (keys %ord) { (my $r = $k) =~ s/[0-9][ab] .*//; $n{$r}++; $b{$r}++ if $foc{$k} && $ord{$k} }
my ($N, $B) = (0, 0);
print "both  ", join(", ", map { $N += $n{$_}; $B += ($b{$_} // 0); sprintf "%s %d/%d", $_, $b{$_} // 0, $n{$_} } sort keys %n), "; all $B of $N\n";
