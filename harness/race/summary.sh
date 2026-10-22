#!/bin/bash
# summary.sh [BUILD]: each line's scores by class (t tabletop, k kiwi servo, b base, d drone, h humanoid), the build's
# alone when BUILD (bd35) is given; entries a VOID line names are left out from its time on.
RACE=${RACE:-/root/race}; B=${1:-}
for L in 1 2 3 4 6; do
  f=$RACE/ledger/line$L.txt; [ -f $f ] || continue
  perl -e '
    my ($b) = @ARGV; my (%void, %n, %pass, %ex);
    while (<STDIN>) {
      if (/^VOID (\d\d:\d\d): entries from (\d\d:\d\d) on for ([^:]+):/) { $void{$_} = $2 for split / /, $3; next; }
      next unless /^(\d\d:\d\d) (([a-z])\d+)_L\d+_\S+ (step\S+) (.*)/;
      my ($t, $name, $c, $step, $rest) = ($1, $2, $3, $4, $5);
      next if exists $void{$name} && $t ge $void{$name};
      next if $b ne "" && $rest !~ /\[$b\]\s*$/;
      $n{$c}++;
      my $ok = $rest =~ /^PASS/ ? 1 : 0;
      $pass{$c} += $ok;
      $ex{$c} = $rest unless $ok;
    }
    my @c = sort keys %n; my $line = join "  ", map { "$_ " . ($pass{$_} // 0) . "/$n{$_}" } @c;
    print "line '$L': $line\n";
  ' "$B" < $f
done
