--  旧名(09-29 的"托住要多紧"):物理检查搬进了 Contact.Wrench(要的动 + 它躺的面,一个规划;旧的那几个名字也在那里,是同一个规划没有面时的那一种)。
--  这里只剩一行改名,给还没换过来的用处(selfcheck.adb 的焊点);合并时主代理把那些用处改成 Contact.Wrench、删掉这个文件
with Contact.Wrench;
package Contact.Hold renames Contact.Wrench;
