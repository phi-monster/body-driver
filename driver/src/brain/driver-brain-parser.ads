--  Reading a program (LANGUAGE.md 11 and 17.2): the text the brain wrote, one
--  statement per line, into a Driver.Brain.Programs tree.
--
--  It accepts everything section 17.2 lists, including forms no keyboard
--  offers, so a person typing as the brain is understood as well. A line it
--  cannot read refuses the whole program before anything moves, with the
--  line, the reason in plain words and a line the brain can write instead.
--
--  A stretch whose constraints end in "<word> up" or "<word> down" is one
--  change of a quantity (LANGUAGE.md 17.1), and everything before that word
--  is the thing's name: the quantity keyboard never offers "and" or the
--  relation words, so a name there may contain them.

with Driver.Brain.Programs;

package Driver.Brain.Parser is

   procedure Parse
     (Text   : String;
      Result : out Driver.Brain.Programs.Program;
      Ok     : out Boolean;
      Why    : out Driver.Brain.Programs.Refusal);

end Driver.Brain.Parser;
