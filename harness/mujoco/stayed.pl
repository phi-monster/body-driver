#!/usr/bin/perl
# stayed.pl RUNS: simulator-side rig code. For every boot, every group's pushes as the driver logged what each left
# changed in each eye (Motion.Note_Stayed: the cells whose picture moved from the push's first frame and stayed moved,
# of the eye's textured cells), beside the truth: whether the group carries the eye, and the lock-in's share at
# recognition (the boot's "group G eye E" lines). One line a group and eye: for each push its sign, the share it left
# moved, the median pixels the resolved ones moved by, and how many moved too far to resolve.
use strict; use warnings; use JSON::PP;
my $runs = shift or die "stayed.pl RUNS\n";
for my $R (sort glob("$runs/*/")) {
  $R =~ s#/$##;
  (my $name = $R) =~ s#.*/##;
  open(my $t, "<", "$R/truth.json") or next; my $truth = do { local $/; decode_json(<$t>) }; close $t;
  my $L = -e "$R/driver.log" ? "$R/driver.log" : "$R/driver.log.zst";
  open(my $f, (-e "$R/driver.log" ? "<" : "-|"), (-e "$R/driver.log" ? $L : "zstd -dc $L")) or next;
  my (%cam_key, %group_key, $in, %stayed, %lockin);
  while (my $line = <$f>) {
    if ($line =~ /\[core\] the robot reports:/) { $in = 1; next; }
    if ($in) {
      if ($line =~ /^camera (\d+): [^\/\s]+\/([^\/\s]+)\//) { $cam_key{$1} = $2; next; }
      if ($line =~ /^group (\d+): (?:[^\/\s]+\/)*([^\/\s,]+),/) { $group_key{$1} = $2; next; }
      $in = 0 if $line =~ /^\s*$/;
      next;
    }
    push @{$stayed{$1}{$3}}, sprintf("%s%.2f/%s/%s", $2 eq "upwards" ? "+" : "-", $5 ? $4 / $5 : 0, $7 // "-", $6 // "-")
      if $line =~ /motion: group (\d+)'s push (upwards|downwards) left eye (\d+) changed: (\d+) of (\d+) textured cells moved and stayed(?:, (\d+) of them too far to resolve, the others by a median of (\S+) px)?/;
    $lockin{$1}{$2} = sprintf("%s %.2f", $3, $5 ? $4 / $5 : 0)
      if $line =~ /boot: group (\d+) eye (\d+): (\w+), (\d+) of (\d+) cells respond/;
  }
  close $f;
  my %group_by_key = map { $_->{wire} => $_ } @{$truth->{groups}};
  my %cam_by_key = map { $_->{wire} => $_ } @{$truth->{cameras}};
  for my $g (sort { $a <=> $b } keys %stayed) {
    my $tg = $group_by_key{$group_key{$g} // ""} or next;
    for my $e (sort { $a <=> $b } keys %{$stayed{$g}}) {
      my $tc = $cam_by_key{$cam_key{$e} // ""} or next;
      printf "%-10s %-12s %-8s %-18s %-16s %-7s stayed %-28s lock-in %s\n", $name, $tg->{name}, $tg->{role}, $tc->{name},
        $tc->{mount}, (defined $tc->{by} && $tc->{by} eq $tg->{name}) ? "CARRIES" : "-", join(" ", @{$stayed{$g}{$e}}),
        $lockin{$g}{$e} // "-";
    }
  }
}
