--  旧名:接触集的搜索搬进了 Contact.Search(身体沿它的路走过去碰到东西 + 物理检查;不叫"抓":捏住只是其中一种)。
--  这里只剩一行改名,给还没换过来的用处(act.ads / act.adb 里 Plan_Contact 的声明、selfcheck.adb 的焊点);
--  合并时主代理把那些用处改成 Contact.Search、删掉这个文件
with Contact.Search;
package Contact.Grasp renames Contact.Search;
