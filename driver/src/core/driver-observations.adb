package body Driver.Observations is

   --  Recognition and parsing are built on Driver.Msgpack (in progress).

   procedure Recognize (Message : Driver.Bytes.Byte_Array; L : out Layout; Ok : out Boolean) is
      pragma Unreferenced (Message);
   begin
      L := (others => <>);
      Ok := False;
   end Recognize;

   procedure Parse
     (Message : Driver.Bytes.Byte_Array;
      L       : Layout;
      Beat    : Driver.Clock.Beat;
      O       : out Observation;
      Ok      : out Boolean)
   is
      pragma Unreferenced (Message, L);
   begin
      O := (Beat => Beat, others => <>);
      Ok := False;
   end Parse;

end Driver.Observations;
