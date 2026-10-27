#!/usr/bin/perl
# order3.pl RUNS: simulator-side rig code. order2.pl (retired 10-27), with an order told only where the arm's last
# fit held: the boot takes an order from a fit that held (Kinematics.Order_Told is Fitted and the result's Order_Told,
# and a fit that failed keeps the last one that held), and step 2's scoring takes an arm by its last verdict
# (focal3.pl). order2.pl took the last "is told" line also where the fit that logged it then failed (round n26s2: G1
# 12a's arm 1 and the Shadow 3b's, both "not fitted" after it) or never ended (RB-Y1 2a's arm 2, the boot's beats out
# within its fit): an order line is its fit's, the verdict line after it says whether the fit held, and a fit begun
# from one of the same estimate logs none (its order is the one it began from).
#
# An order line names the joints the fit had by their places among them (Fit's own numbering, 1 to its count), and
# the fit has the arm's joints less any it leaves out (Kinematics: Index, the fitted readings in their order): the
# fitted line names them by their readings ("its joints from the base, by their readings: ..."), the fitted ones
# first in the chain's order and those left out after them. The truth is the readings in the true order (the
# scramble's model joints from the base) kept to the fitted readings and named by their places among them, as the
# order line names them. Until 10-27 the truth was the readings kept to the order line's numbers taken as readings:
# right only where the fit left out none but the last readings (n30s2's UR5e 1a, reading 4 left out: its order 1 2 5
# 3 6 by readings, the true one, scored WRONG against 1 2 5 3 4).
use strict; use warnings; use JSON::PP;
my $runs = shift or die "order3.pl RUNS\n";
my (%arms, %told, %right, %both, %failed_told);
for my $R (sort glob("$runs/*/")) {
  $R =~ s#/$##; (my $name = $R) =~ s#.*/##; (my $kind = $name) =~ s/[123][ab]$//;
  next unless -s "$R/truth.json" && -s "$R/scramble.json";
  open(my $t, "<", "$R/truth.json") or next; my $truth = do { local $/; decode_json(<$t>) }; close $t;
  open(my $s, "<", "$R/scramble.json") or next; my $scr = do { local $/; decode_json(<$s>) }; close $s;
  my $L = -e "$R/driver.log" ? "$R/driver.log" : "$R/driver.log.zst";
  open(my $f, (-e "$R/driver.log" ? "<" : "-|"), (-e "$R/driver.log" ? $L : "zstd -dc $L")) or next;
  my (%group_key, $in, %sweep, %order, %verdict, %held, %last_line, %index);
  while (my $l = <$f>) {
    if ($l =~ /\[core\] the robot reports:/) { $in = 1; next; }
    if ($in) {
      if ($l =~ /^group (\d+): (?:[^\/\s]+\/)*([^\/\s,]+),/) { $group_key{$1} = $2; next; }
      $in = 0 if $l =~ /^\s*$/ && %group_key;
    }
    $sweep{$2} = $3 if $l =~ /^\@(\d+) \[robot\] boot: sweeping arm (\d+) \(group (\d+)\)/;
    if ($l =~ /kinematics: the order of the joints along arm (\d+), ([\d ]+), is (NOT )?told/) { $last_line{$1} = [$2, $3 ? 0 : 1] }
    if ($l =~ /kinematics: arm (\d+) (fitted from|not fitted)/) {
      my $an = $1;
      $verdict{$an} = $2 eq "fitted from" ? 1 : 0;
      $held{$an} = $verdict{$an} ? $last_line{$an} : undef;
      # The fitted readings: the chain's first as many as the order line names, in ascending order (Index).
      if ($verdict{$an} && $last_line{$an} && $l =~ /by their readings: ([\d, ]+?);/) {
        my @chain = split /,\s*/, $1;
        my @fit = split " ", $last_line{$an}[0];
        if (@fit <= @chain) {
          my @idx = sort { $a <=> $b } @chain[0 .. $#fit];
          my $named = join(" ", map { $idx[$_ - 1] // "?" } @fit);
          if ($named ne join(" ", @chain[0 .. $#fit])) {
            print STDERR "$name arm $an: the order line $last_line{$an}[0] names readings $named, the fitted line @chain[0 .. $#fit]\n";
          }
          $index{$an} = \@idx;
        }
      }
    }
  }
  close $f;
  for my $a (keys %sweep) { $order{$a} = $held{$a} // ($last_line{$a} ? [$last_line{$a}[0], $last_line{$a}[1]] : undef) }
  my %gname = map { $_->{wire} => $_->{name} } @{$truth->{groups}};
  for my $arm (sort { $a <=> $b } keys %sweep) {
    my $gn = $gname{$group_key{$sweep{$arm}} // ""} // "g$sweep{$arm}";
    my $g = $scr->{groups}{$gn};
    $arms{$kind}++;
    unless ($order{$arm} && $g) { printf "%-10s arm %d %-12s no order\n", $name, $arm, $gn; next; }
    my @o = @{$g->{order}};
    my %chan_of = map { $o[$_] => $_ + 1 } 0 .. $#o;   # model joint -> channel (1-based)
    my @true = map { $chan_of{$_} } sort { $a <=> $b } keys %chan_of;
    my @fit = split " ", $order{$arm}[0];
    # The fitted readings, by the last fit that held; where none held with as many joints, readings 1 to the count.
    my @idx = $index{$arm} && @{$index{$arm}} == @fit ? @{$index{$arm}} : (1 .. scalar @fit);
    my %place = map { $idx[$_] => $_ + 1 } 0 .. $#idx;   # reading -> its place among the fitted
    my @true_here = map { $place{$_} } grep { exists $place{$_} } @true;   # the fitted joints in the true order
    my $ok = join(" ", @fit) eq join(" ", @true_here);
    my $held = $verdict{$arm} // 0;
    my $tl = $order{$arm}[1] && $held && $held{$arm};
    $failed_told{$kind}++ if $order{$arm}[1] && !$tl;
    $told{$kind}++ if $tl; $right{$kind}++ if $ok; $both{$kind}++ if $ok && $tl;
    printf "%-10s arm %d %-12s fit %-16s truth %-16s %s %s\n", $name, $arm, $gn, join(" ", @fit), join(" ", @true_here),
      $ok ? "RIGHT" : "WRONG", $tl ? "told" : ($order{$arm}[1] ? "untold (told by a fit that then failed)" : "untold");
    if (join(" ", @idx) ne join(" ", 1 .. scalar @fit)) {
      my %fitted = map { $_ => 1 } @idx;
      printf "    by readings: fit %s, truth %s; readings %s left out of the fit\n", join(" ", map { $idx[$_ - 1] } @fit),
        join(" ", grep { $fitted{$_} } @true), join(" ", grep { !$fitted{$_} } 1 .. scalar @true);
    }
  }
}
for my $k (sort keys %arms) {
  printf "%-8s arms swept %2d, order told %2d, right %2d, right and told %2d; told by a fit that then failed %d\n", $k,
    $arms{$k}, $told{$k} // 0, $right{$k} // 0, $both{$k} // 0, $failed_told{$k} // 0;
}
