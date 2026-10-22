#!/bin/bash
# compare1.sh LEDGER...: step 1 verdicts of the same seeds in several ledgers (the first the baseline), by class, and
# every seed whose verdict changed from the first ledger to the others.
perl -e '
  my @files = @ARGV; my %v; my %seen;
  for my $i (0 .. $#files) {
    open(my $h, "<", $files[$i]) or next;
    while (<$h>) { next unless /^\S+ ([a-z]\d+)_L1_\S+ step1 (PASS|FAIL)/; $v{$1}[$i] = $2; $seen{$1}++; }
  }
  my @both = grep { my $n = $_; !grep { !defined $v{$n}[$_] } 0 .. $#files } sort keys %v;
  my %count;
  for my $n (@both) { my $c = substr($n, 0, 1); for my $i (0 .. $#files) { $count{$c}[$i]{n}++; $count{$c}[$i]{p}++ if $v{$n}[$i] eq "PASS"; } }
  print "seeds in every ledger: ", scalar(@both), "\n";
  for my $c (sort keys %count) { print "  $c: ", join("  ->  ", map { ($count{$c}[$_]{p} // 0) . "/" . $count{$c}[$_]{n} } 0 .. $#files), "\n"; }
  for my $n (@both) { my @x = map { $v{$n}[$_] } 0 .. $#files; next unless grep { $_ ne $x[0] } @x; print "  changed $n: @x\n"; }
' "$@"
