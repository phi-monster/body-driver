pragma Warnings (Off);
pragma Ada_95;
pragma Source_File_Name (ada_main, Spec_File_Name => "b__body_driver.ads");
pragma Source_File_Name (ada_main, Body_File_Name => "b__body_driver.adb");
pragma Suppress (Overflow_Check);
with Ada.Exceptions;

package body ada_main is

   E016 : Short_Integer; pragma Import (Ada, E016, "ada__exceptions_E");
   E012 : Short_Integer; pragma Import (Ada, E012, "system__soft_links_E");
   E010 : Short_Integer; pragma Import (Ada, E010, "system__exception_table_E");
   E019 : Short_Integer; pragma Import (Ada, E019, "system__exceptions_E");
   E050 : Short_Integer; pragma Import (Ada, E050, "system__soft_links__initialize_E");
   E131 : Short_Integer; pragma Import (Ada, E131, "ada__assertions_E");
   E007 : Short_Integer; pragma Import (Ada, E007, "ada__containers_E");
   E058 : Short_Integer; pragma Import (Ada, E058, "ada__io_exceptions_E");
   E094 : Short_Integer; pragma Import (Ada, E094, "ada__numerics_E");
   E061 : Short_Integer; pragma Import (Ada, E061, "ada__strings_E");
   E063 : Short_Integer; pragma Import (Ada, E063, "ada__strings__utf_encoding_E");
   E235 : Short_Integer; pragma Import (Ada, E235, "gnat_E");
   E093 : Short_Integer; pragma Import (Ada, E093, "interfaces__c_E");
   E185 : Short_Integer; pragma Import (Ada, E185, "interfaces__c__strings_E");
   E119 : Short_Integer; pragma Import (Ada, E119, "system__os_lib_E");
   E071 : Short_Integer; pragma Import (Ada, E071, "ada__tags_E");
   E060 : Short_Integer; pragma Import (Ada, E060, "ada__strings__text_buffers_E");
   E057 : Short_Integer; pragma Import (Ada, E057, "ada__streams_E");
   E125 : Short_Integer; pragma Import (Ada, E125, "system__file_control_block_E");
   E087 : Short_Integer; pragma Import (Ada, E087, "system__finalization_root_E");
   E055 : Short_Integer; pragma Import (Ada, E055, "ada__finalization_E");
   E111 : Short_Integer; pragma Import (Ada, E111, "system__file_io_E");
   E208 : Short_Integer; pragma Import (Ada, E208, "ada__streams__stream_io_E");
   E181 : Short_Integer; pragma Import (Ada, E181, "system__storage_pools_E");
   E214 : Short_Integer; pragma Import (Ada, E214, "system__storage_pools__subpools_E");
   E137 : Short_Integer; pragma Import (Ada, E137, "ada__calendar_E");
   E244 : Short_Integer; pragma Import (Ada, E244, "ada__calendar__delays_E");
   E145 : Short_Integer; pragma Import (Ada, E145, "ada__calendar__time_zones_E");
   E105 : Short_Integer; pragma Import (Ada, E105, "ada__text_io_E");
   E266 : Short_Integer; pragma Import (Ada, E266, "gnat__secure_hashes_E");
   E268 : Short_Integer; pragma Import (Ada, E268, "gnat__secure_hashes__sha1_E");
   E264 : Short_Integer; pragma Import (Ada, E264, "gnat__sha1_E");
   E156 : Short_Integer; pragma Import (Ada, E156, "ada__strings__maps_E");
   E159 : Short_Integer; pragma Import (Ada, E159, "ada__strings__maps__constants_E");
   E169 : Short_Integer; pragma Import (Ada, E169, "ada__strings__unbounded_E");
   E228 : Short_Integer; pragma Import (Ada, E228, "system__pool_global_E");
   E237 : Short_Integer; pragma Import (Ada, E237, "gnat__sockets_E");
   E240 : Short_Integer; pragma Import (Ada, E240, "gnat__sockets__poll_E");
   E250 : Short_Integer; pragma Import (Ada, E250, "gnat__sockets__thin_common_E");
   E242 : Short_Integer; pragma Import (Ada, E242, "gnat__sockets__thin_E");
   E179 : Short_Integer; pragma Import (Ada, E179, "system__regexp_E");
   E135 : Short_Integer; pragma Import (Ada, E135, "ada__directories_E");
   E127 : Short_Integer; pragma Import (Ada, E127, "backup_E");
   E212 : Short_Integer; pragma Import (Ada, E212, "bytes_E");
   E133 : Short_Integer; pragma Import (Ada, E133, "codec_E");
   E222 : Short_Integer; pragma Import (Ada, E222, "draw_E");
   E224 : Short_Integer; pragma Import (Ada, E224, "flow_E");
   E234 : Short_Integer; pragma Import (Ada, E234, "http_client_E");
   E252 : Short_Integer; pragma Import (Ada, E252, "json_E");
   E232 : Short_Integer; pragma Import (Ada, E232, "brain_E");
   E275 : Short_Integer; pragma Import (Ada, E275, "memory_E");
   E226 : Short_Integer; pragma Import (Ada, E226, "monitor_E");
   E260 : Short_Integer; pragma Import (Ada, E260, "msgpack_E");
   E258 : Short_Integer; pragma Import (Ada, E258, "layout_E");
   E277 : Short_Integer; pragma Import (Ada, E277, "picture_E");
   E273 : Short_Integer; pragma Import (Ada, E273, "table_E");
   E262 : Short_Integer; pragma Import (Ada, E262, "websocket_E");
   E256 : Short_Integer; pragma Import (Ada, E256, "plug_E");
   E254 : Short_Integer; pragma Import (Ada, E254, "chan_E");
   E279 : Short_Integer; pragma Import (Ada, E279, "schema_E");
   E281 : Short_Integer; pragma Import (Ada, E281, "selfmap_E");
   E283 : Short_Integer; pragma Import (Ada, E283, "world_E");
   E285 : Short_Integer; pragma Import (Ada, E285, "zone_E");
   E005 : Short_Integer; pragma Import (Ada, E005, "act_E");
   E289 : Short_Integer; pragma Import (Ada, E289, "bodyfile_E");

   Sec_Default_Sized_Stacks : array (1 .. 1) of aliased System.Secondary_Stack.SS_Stack (System.Parameters.Runtime_Default_Sec_Stack_Size);

   Local_Priority_Specific_Dispatching : constant String := "";
   Local_Interrupt_States : constant String := "";

   Is_Elaborated : Boolean := False;

   procedure finalize_library is
   begin
      declare
         procedure F1;
         pragma Import (Ada, F1, "act__finalize_body");
      begin
         E005 := E005 - 1;
         F1;
      end;
      declare
         procedure F2;
         pragma Import (Ada, F2, "act__finalize_spec");
      begin
         F2;
      end;
      E285 := E285 - 1;
      declare
         procedure F3;
         pragma Import (Ada, F3, "zone__finalize_spec");
      begin
         F3;
      end;
      E283 := E283 - 1;
      declare
         procedure F4;
         pragma Import (Ada, F4, "world__finalize_spec");
      begin
         F4;
      end;
      E281 := E281 - 1;
      declare
         procedure F5;
         pragma Import (Ada, F5, "selfmap__finalize_spec");
      begin
         F5;
      end;
      E279 := E279 - 1;
      declare
         procedure F6;
         pragma Import (Ada, F6, "schema__finalize_spec");
      begin
         F6;
      end;
      E256 := E256 - 1;
      declare
         procedure F7;
         pragma Import (Ada, F7, "plug__finalize_spec");
      begin
         F7;
      end;
      E273 := E273 - 1;
      declare
         procedure F8;
         pragma Import (Ada, F8, "table__finalize_spec");
      begin
         F8;
      end;
      E277 := E277 - 1;
      declare
         procedure F9;
         pragma Import (Ada, F9, "picture__finalize_spec");
      begin
         F9;
      end;
      E258 := E258 - 1;
      declare
         procedure F10;
         pragma Import (Ada, F10, "layout__finalize_spec");
      begin
         F10;
      end;
      E260 := E260 - 1;
      declare
         procedure F11;
         pragma Import (Ada, F11, "msgpack__finalize_spec");
      begin
         F11;
      end;
      E232 := E232 - 1;
      declare
         procedure F12;
         pragma Import (Ada, F12, "brain__finalize_spec");
      begin
         F12;
      end;
      E252 := E252 - 1;
      declare
         procedure F13;
         pragma Import (Ada, F13, "json__finalize_spec");
      begin
         F13;
      end;
      E212 := E212 - 1;
      declare
         procedure F14;
         pragma Import (Ada, F14, "bytes__finalize_spec");
      begin
         F14;
      end;
      declare
         procedure F15;
         pragma Import (Ada, F15, "ada__directories__finalize_body");
      begin
         E135 := E135 - 1;
         F15;
      end;
      declare
         procedure F16;
         pragma Import (Ada, F16, "ada__directories__finalize_spec");
      begin
         F16;
      end;
      E179 := E179 - 1;
      declare
         procedure F17;
         pragma Import (Ada, F17, "system__regexp__finalize_spec");
      begin
         F17;
      end;
      declare
         procedure F18;
         pragma Import (Ada, F18, "gnat__sockets__finalize_body");
      begin
         E237 := E237 - 1;
         F18;
      end;
      declare
         procedure F19;
         pragma Import (Ada, F19, "gnat__sockets__finalize_spec");
      begin
         F19;
      end;
      E228 := E228 - 1;
      declare
         procedure F20;
         pragma Import (Ada, F20, "system__pool_global__finalize_spec");
      begin
         F20;
      end;
      E169 := E169 - 1;
      declare
         procedure F21;
         pragma Import (Ada, F21, "ada__strings__unbounded__finalize_spec");
      begin
         F21;
      end;
      E264 := E264 - 1;
      declare
         procedure F22;
         pragma Import (Ada, F22, "gnat__sha1__finalize_spec");
      begin
         F22;
      end;
      E105 := E105 - 1;
      declare
         procedure F23;
         pragma Import (Ada, F23, "ada__text_io__finalize_spec");
      begin
         F23;
      end;
      E214 := E214 - 1;
      declare
         procedure F24;
         pragma Import (Ada, F24, "system__storage_pools__subpools__finalize_spec");
      begin
         F24;
      end;
      E208 := E208 - 1;
      declare
         procedure F25;
         pragma Import (Ada, F25, "ada__streams__stream_io__finalize_spec");
      begin
         F25;
      end;
      declare
         procedure F26;
         pragma Import (Ada, F26, "system__file_io__finalize_body");
      begin
         E111 := E111 - 1;
         F26;
      end;
      declare
         procedure Reraise_Library_Exception_If_Any;
            pragma Import (Ada, Reraise_Library_Exception_If_Any, "__gnat_reraise_library_exception_if_any");
      begin
         Reraise_Library_Exception_If_Any;
      end;
   end finalize_library;

   procedure adafinal is
      procedure s_stalib_adafinal;
      pragma Import (Ada, s_stalib_adafinal, "system__standard_library__adafinal");

      procedure Runtime_Finalize;
      pragma Import (C, Runtime_Finalize, "__gnat_runtime_finalize");

   begin
      if not Is_Elaborated then
         return;
      end if;
      Is_Elaborated := False;
      Runtime_Finalize;
      s_stalib_adafinal;
   end adafinal;

   type No_Param_Proc is access procedure;
   pragma Favor_Top_Level (No_Param_Proc);

   procedure adainit is
      Main_Priority : Integer;
      pragma Import (C, Main_Priority, "__gl_main_priority");
      Time_Slice_Value : Integer;
      pragma Import (C, Time_Slice_Value, "__gl_time_slice_val");
      WC_Encoding : Character;
      pragma Import (C, WC_Encoding, "__gl_wc_encoding");
      Locking_Policy : Character;
      pragma Import (C, Locking_Policy, "__gl_locking_policy");
      Queuing_Policy : Character;
      pragma Import (C, Queuing_Policy, "__gl_queuing_policy");
      Task_Dispatching_Policy : Character;
      pragma Import (C, Task_Dispatching_Policy, "__gl_task_dispatching_policy");
      Priority_Specific_Dispatching : System.Address;
      pragma Import (C, Priority_Specific_Dispatching, "__gl_priority_specific_dispatching");
      Num_Specific_Dispatching : Integer;
      pragma Import (C, Num_Specific_Dispatching, "__gl_num_specific_dispatching");
      Main_CPU : Integer;
      pragma Import (C, Main_CPU, "__gl_main_cpu");
      Interrupt_States : System.Address;
      pragma Import (C, Interrupt_States, "__gl_interrupt_states");
      Num_Interrupt_States : Integer;
      pragma Import (C, Num_Interrupt_States, "__gl_num_interrupt_states");
      Unreserve_All_Interrupts : Integer;
      pragma Import (C, Unreserve_All_Interrupts, "__gl_unreserve_all_interrupts");
      Exception_Tracebacks : Integer;
      pragma Import (C, Exception_Tracebacks, "__gl_exception_tracebacks");
      Exception_Tracebacks_Symbolic : Integer;
      pragma Import (C, Exception_Tracebacks_Symbolic, "__gl_exception_tracebacks_symbolic");
      Detect_Blocking : Integer;
      pragma Import (C, Detect_Blocking, "__gl_detect_blocking");
      Default_Stack_Size : Integer;
      pragma Import (C, Default_Stack_Size, "__gl_default_stack_size");
      Default_Secondary_Stack_Size : System.Parameters.Size_Type;
      pragma Import (C, Default_Secondary_Stack_Size, "__gnat_default_ss_size");
      Bind_Env_Addr : System.Address;
      pragma Import (C, Bind_Env_Addr, "__gl_bind_env_addr");
      Interrupts_Default_To_System : Integer;
      pragma Import (C, Interrupts_Default_To_System, "__gl_interrupts_default_to_system");

      procedure Runtime_Initialize (Install_Handler : Integer);
      pragma Import (C, Runtime_Initialize, "__gnat_runtime_initialize");

      Finalize_Library_Objects : No_Param_Proc;
      pragma Import (C, Finalize_Library_Objects, "__gnat_finalize_library_objects");
      Binder_Sec_Stacks_Count : Natural;
      pragma Import (Ada, Binder_Sec_Stacks_Count, "__gnat_binder_ss_count");
      Default_Sized_SS_Pool : System.Address;
      pragma Import (Ada, Default_Sized_SS_Pool, "__gnat_default_ss_pool");

   begin
      if Is_Elaborated then
         return;
      end if;
      Is_Elaborated := True;
      Main_Priority := -1;
      Time_Slice_Value := -1;
      WC_Encoding := 'b';
      Locking_Policy := ' ';
      Queuing_Policy := ' ';
      Task_Dispatching_Policy := ' ';
      Priority_Specific_Dispatching :=
        Local_Priority_Specific_Dispatching'Address;
      Num_Specific_Dispatching := 0;
      Main_CPU := -1;
      Interrupt_States := Local_Interrupt_States'Address;
      Num_Interrupt_States := 0;
      Unreserve_All_Interrupts := 0;
      Exception_Tracebacks := 1;
      Exception_Tracebacks_Symbolic := 1;
      Detect_Blocking := 0;
      Default_Stack_Size := -1;

      ada_main'Elab_Body;
      Default_Secondary_Stack_Size := System.Parameters.Runtime_Default_Sec_Stack_Size;
      Binder_Sec_Stacks_Count := 1;
      Default_Sized_SS_Pool := Sec_Default_Sized_Stacks'Address;

      Runtime_Initialize (1);

      Finalize_Library_Objects := finalize_library'access;

      Ada.Exceptions'Elab_Spec;
      System.Soft_Links'Elab_Spec;
      System.Exception_Table'Elab_Body;
      E010 := E010 + 1;
      System.Exceptions'Elab_Spec;
      E019 := E019 + 1;
      System.Soft_Links.Initialize'Elab_Body;
      E050 := E050 + 1;
      E012 := E012 + 1;
      E016 := E016 + 1;
      Ada.Assertions'Elab_Spec;
      E131 := E131 + 1;
      Ada.Containers'Elab_Spec;
      E007 := E007 + 1;
      Ada.Io_Exceptions'Elab_Spec;
      E058 := E058 + 1;
      Ada.Numerics'Elab_Spec;
      E094 := E094 + 1;
      Ada.Strings'Elab_Spec;
      E061 := E061 + 1;
      Ada.Strings.Utf_Encoding'Elab_Spec;
      E063 := E063 + 1;
      Gnat'Elab_Spec;
      E235 := E235 + 1;
      Interfaces.C'Elab_Spec;
      E093 := E093 + 1;
      Interfaces.C.Strings'Elab_Spec;
      E185 := E185 + 1;
      System.Os_Lib'Elab_Body;
      E119 := E119 + 1;
      Ada.Tags'Elab_Spec;
      Ada.Tags'Elab_Body;
      E071 := E071 + 1;
      Ada.Strings.Text_Buffers'Elab_Spec;
      E060 := E060 + 1;
      Ada.Streams'Elab_Spec;
      E057 := E057 + 1;
      System.File_Control_Block'Elab_Spec;
      E125 := E125 + 1;
      System.Finalization_Root'Elab_Spec;
      E087 := E087 + 1;
      Ada.Finalization'Elab_Spec;
      E055 := E055 + 1;
      System.File_Io'Elab_Body;
      E111 := E111 + 1;
      Ada.Streams.Stream_Io'Elab_Spec;
      E208 := E208 + 1;
      System.Storage_Pools'Elab_Spec;
      E181 := E181 + 1;
      System.Storage_Pools.Subpools'Elab_Spec;
      E214 := E214 + 1;
      Ada.Calendar'Elab_Spec;
      Ada.Calendar'Elab_Body;
      E137 := E137 + 1;
      Ada.Calendar.Delays'Elab_Body;
      E244 := E244 + 1;
      Ada.Calendar.Time_Zones'Elab_Spec;
      E145 := E145 + 1;
      Ada.Text_Io'Elab_Spec;
      Ada.Text_Io'Elab_Body;
      E105 := E105 + 1;
      E266 := E266 + 1;
      E268 := E268 + 1;
      Gnat.Sha1'Elab_Spec;
      E264 := E264 + 1;
      Ada.Strings.Maps'Elab_Spec;
      E156 := E156 + 1;
      Ada.Strings.Maps.Constants'Elab_Spec;
      E159 := E159 + 1;
      Ada.Strings.Unbounded'Elab_Spec;
      E169 := E169 + 1;
      System.Pool_Global'Elab_Spec;
      E228 := E228 + 1;
      Gnat.Sockets'Elab_Spec;
      Gnat.Sockets.Thin_Common'Elab_Spec;
      E250 := E250 + 1;
      E242 := E242 + 1;
      Gnat.Sockets'Elab_Body;
      E237 := E237 + 1;
      E240 := E240 + 1;
      System.Regexp'Elab_Spec;
      E179 := E179 + 1;
      Ada.Directories'Elab_Spec;
      Ada.Directories'Elab_Body;
      E135 := E135 + 1;
      E127 := E127 + 1;
      Bytes'Elab_Spec;
      E212 := E212 + 1;
      E133 := E133 + 1;
      E222 := E222 + 1;
      E224 := E224 + 1;
      E234 := E234 + 1;
      Json'Elab_Spec;
      E252 := E252 + 1;
      Brain'Elab_Spec;
      E232 := E232 + 1;
      E275 := E275 + 1;
      E226 := E226 + 1;
      Msgpack'Elab_Spec;
      E260 := E260 + 1;
      Layout'Elab_Spec;
      E258 := E258 + 1;
      Picture'Elab_Spec;
      E277 := E277 + 1;
      Table'Elab_Spec;
      E273 := E273 + 1;
      E262 := E262 + 1;
      Plug'Elab_Spec;
      E256 := E256 + 1;
      E254 := E254 + 1;
      Schema'Elab_Spec;
      E279 := E279 + 1;
      Selfmap'Elab_Spec;
      E281 := E281 + 1;
      World'Elab_Spec;
      E283 := E283 + 1;
      Zone'Elab_Spec;
      E285 := E285 + 1;
      Act'Elab_Spec;
      Act'Elab_Body;
      E005 := E005 + 1;
      E289 := E289 + 1;
   end adainit;

   procedure Ada_Main_Program;
   pragma Import (Ada, Ada_Main_Program, "_ada_body_driver");

   function main
     (argc : Integer;
      argv : System.Address;
      envp : System.Address)
      return Integer
   is
      procedure Initialize (Addr : System.Address);
      pragma Import (C, Initialize, "__gnat_initialize");

      procedure Finalize;
      pragma Import (C, Finalize, "__gnat_finalize");
      SEH : aliased array (1 .. 2) of Integer;

      Ensure_Reference : aliased System.Address := Ada_Main_Program_Name'Address;
      pragma Volatile (Ensure_Reference);

   begin
      if gnat_argc = 0 then
         gnat_argc := argc;
         gnat_argv := argv;
      end if;
      gnat_envp := envp;

      Initialize (SEH'Address);
      adainit;
      Ada_Main_Program;
      adafinal;
      Finalize;
      return (gnat_exit_status);
   end;

--  BEGIN Object file/option list
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/backup.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/bytes.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/codec.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/draw.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/flow.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/http_client.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/json.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/brain.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/memory.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/monitor.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/msgpack.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/layout.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/picture.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/table.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/websocket.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/plug.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/chan.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/schema.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/selfmap.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/world.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/zone.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/act.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/bodyfile.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/body_driver.o
   --   -L/Users/gogogod/Project/phi/body-driver/driver/obj/
   --   -L/Users/gogogod/Project/phi/body-driver/driver/obj/
   --   -L/users/gogogod/.local/share/alire/toolchains/gnat_native_16.1.0_657cf254/lib/gcc/aarch64-apple-darwin24.6.0/16.1.0/adalib/
   --   -static
   --   -lgnat
   --   -lm
--  END Object file/option list   

end ada_main;
