--  What the process holds in memory, for the log: the resident set the
--  kernel reports (Linux: /proc/self/status, VmRSS), in kilobytes; 0 where
--  the kernel does not report it.
package Driver.Memory is

   function Resident_Kilobytes return Natural;

end Driver.Memory;
