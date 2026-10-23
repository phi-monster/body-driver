#!/usr/bin/perl
# verdicts.pl RUNS: simulator-side rig code. Every group in every eye as the driver judged it at recognition (the boot's
# "group G eye E" lines), beside the truth: whether the group truly carries the eye (the camera's "by"). One line each:
# run, group (truth name, role), eye (truth name, mount), carries or not, the driver's verdict, cells responding of
# textured, the rest's pull along the push and its motion together (sigmas). For studying what tells an eye's own
# motion from a part moving in its view.
use strict; use warnings; use JSON::PP;
my $runs = shift or die "verdicts.pl RUNS\n";
for my $R (sort glob("$runs/*/")) {
  $R =~ s#/$##;
  (my $name = $R) =~ s#.*/##;
  open(my $t, "<", "$R/truth.json") or next; my $truth = do { local $/; decode_json(<$t>) }; close $t;
  my $L = -e "$R/driver.log" ? "$R/driver.log" : "$R/driver.log.zst";
  open(my $f, (-e "$R/driver.log" ? "<" : "-|"), (-e "$R/driver.log" ? $L : "zstd -dc $L")) or next;
  my (%cam_key, %group_key, $in_report, @rows);
  while (my $line = <$f>) {
    if ($line =~ /\[core\] the robot reports:/) { $in_report = 1; next; }
    if ($in_report) {
      if ($line =~ /^camera (\d+): [^\/\s]+\/([^\/\s]+)\//) { $cam_key{$1} = $2; next; }
      if ($line =~ /^group (\d+): (?:[^\/\s]+\/)*([^\/\s,]+),/) { $group_key{$1} = $2; next; }
      $in_report = 0 if $line =~ /^\s*$/;
      next;
    }
    push @rows, [$1, $2, $3, $4, $5, $6, $7]
      if $line =~ /boot: group (\d+) eye (\d+): (\w+), (\d+) of (\d+) cells respond, the rest pulled along the push by (\S+) sigmas, moving by (\S+)/;
  }
  close $f;
  my %group_by_key = map { $_->{wire} => $_ } @{$truth->{groups}};
  my %cam_by_key = map { $_->{wire} => $_ } @{$truth->{cameras}};
  for my $r (@rows) {
    my ($g, $e, $verdict, $n, $tx, $pull, $rest) = @$r;
    my $tg = $group_by_key{$group_key{$g} // ""}; my $tc = $cam_by_key{$cam_key{$e} // ""};
    next unless $tg && $tc;
    my $carries = defined $tc->{by} && $tc->{by} eq $tg->{name} ? "CARRIES" : "-";
    printf "%-10s %-12s %-8s %-18s %-16s %-7s %-9s %3d/%3d pull %7s rest %7s\n", $name, $tg->{name}, $tg->{role},
      $tc->{name}, $tc->{mount}, $carries, $verdict, $n, $tx, $pull, $rest;
  }
}
