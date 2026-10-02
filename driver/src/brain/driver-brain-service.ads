--  The two questions the body asks the brain service (docs/brain-service.md),
--  through Driver.Services.
--
--  Write a program: one streamed request per round, with this round's
--  keyboard as the grammar for constrained decoding; the answer is read as it
--  arrives and reading stops as soon as Driver.Brain.Runaway says it may.
--  Where is it: one request per name and eye, answered with a box.
--
--  The driver chooses no sampling setting. The deployment gives them as one
--  JSON object in the environment variable BL_BRAIN_SAMPLING, merged as it is
--  into both questions; an object that is not JSON, or that sets a member the
--  driver writes itself, is not used, and the log says why once.

with Ada.Strings.Unbounded;
with Driver.Brain.Keyboard;
with Driver.Brain.Names;
with Driver.Images;

package Driver.Brain.Service is

   use Ada.Strings.Unbounded;

   type Reading_End is (Ended, Complete, Ran_Away, Service_Stopped, Failed);
   --  Ended            the service ended the answer by itself
   --  Complete         a done outside every block was read; the rest was not
   --  Ran_Away         the answer started a copy loop and was cut
   --  Service_Stopped  the service stopped it for its own limit (finish_reason length)
   --  Failed           no answer: the service could not be reached or replied with an error

   function Image (R : Reading_End) return String is
     (case R is
         when Ended           => "ended by itself",
         when Complete        => "complete",
         when Ran_Away        => "ran away",
         when Service_Stopped => "stopped by the service",
         when Failed          => "failed");

   type Answer is record
      How     : Reading_End := Failed;
      Program : Unbounded_String;   --  the finished lines kept, each ending in a line break
      Why     : Unbounded_String;   --  why reading stopped, or what failed
      Seconds : Duration := 0.0;
   end record;

   function Write_Program
     (Picture : Driver.Images.Image;
      Prompt  : String;
      Keys    : Driver.Brain.Keyboard.Keyboard) return Answer
     with Pre => not Driver.Images.Is_Empty (Picture);

   procedure Ask_Where
     (Picture : Driver.Images.Image;
      Name    : String;
      Found   : out Driver.Brain.Names.Pointing;
      Where   : out Driver.Brain.Names.Box;
      Why     : out Unbounded_String)
     with Pre => not Driver.Images.Is_Empty (Picture);
   --  Picture is one eye's own picture, nothing drawn on it.

   function Program_Request (Picture_Url, Prompt, Grammar : String) return String;
   function Where_Request (Picture_Url, Name : String) return String;
   --  The request bodies, exactly as sent.

   function Where_Answer_Grammar return String;
   --  The grammar of the where-is-it answer: {"found":true|false,"bbox_2d":[e,e,e,e]}
   --  with every edge a whole number from 0 to 1000 and no blank anywhere.

   procedure Read_Event (Event : String; Text : in out Unbounded_String; Finish : in out Unbounded_String);
   --  One server-sent event of a streamed answer: its text is appended and
   --  its finish reason, when it has one, kept.

   procedure Read_Where
     (Reply  : String;
      Width  : Positive;
      Height : Positive;
      Found  : out Driver.Brain.Names.Pointing;
      Where  : out Driver.Brain.Names.Box;
      Why    : out Unbounded_String);
   --  The answer to where-is-it: a box in thousandths of the picture,
   --  turned into pixels of a picture of that size.

   procedure New_Episode;
   --  Starts counting the calls of a new episode; the log says how many the
   --  last one made.

private

   function Members (Raw : String; Why : out Unbounded_String) return String;
   --  The members of the deployment's object, without its braces, ready to
   --  merge; "" with Why set when they cannot be used.

end Driver.Brain.Service;
