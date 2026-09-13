pragma Warnings (Off);
pragma Ada_95;
pragma Source_File_Name (ada_main, Spec_File_Name => "b__selfcheck.ads");
pragma Source_File_Name (ada_main, Body_File_Name => "b__selfcheck.adb");
pragma Suppress (Overflow_Check);
with Ada.Exceptions;

package body ada_main is

   E014 : Short_Integer; pragma Import (Ada, E014, "ada__exceptions_E");
   E010 : Short_Integer; pragma Import (Ada, E010, "system__soft_links_E");
   E008 : Short_Integer; pragma Import (Ada, E008, "system__exception_table_E");
   E017 : Short_Integer; pragma Import (Ada, E017, "system__exceptions_E");
   E048 : Short_Integer; pragma Import (Ada, E048, "system__soft_links__initialize_E");
   E135 : Short_Integer; pragma Import (Ada, E135, "ada__assertions_E");
   E005 : Short_Integer; pragma Import (Ada, E005, "ada__containers_E");
   E090 : Short_Integer; pragma Import (Ada, E090, "ada__io_exceptions_E");
   E160 : Short_Integer; pragma Import (Ada, E160, "ada__numerics_E");
   E051 : Short_Integer; pragma Import (Ada, E051, "ada__strings_E");
   E055 : Short_Integer; pragma Import (Ada, E055, "ada__strings__utf_encoding_E");
   E233 : Short_Integer; pragma Import (Ada, E233, "gnat_E");
   E080 : Short_Integer; pragma Import (Ada, E080, "interfaces__c_E");
   E205 : Short_Integer; pragma Import (Ada, E205, "interfaces__c__strings_E");
   E123 : Short_Integer; pragma Import (Ada, E123, "system__os_lib_E");
   E063 : Short_Integer; pragma Import (Ada, E063, "ada__tags_E");
   E053 : Short_Integer; pragma Import (Ada, E053, "ada__strings__text_buffers_E");
   E089 : Short_Integer; pragma Import (Ada, E089, "ada__streams_E");
   E129 : Short_Integer; pragma Import (Ada, E129, "system__file_control_block_E");
   E092 : Short_Integer; pragma Import (Ada, E092, "system__finalization_root_E");
   E087 : Short_Integer; pragma Import (Ada, E087, "ada__finalization_E");
   E119 : Short_Integer; pragma Import (Ada, E119, "system__file_io_E");
   E220 : Short_Integer; pragma Import (Ada, E220, "ada__streams__stream_io_E");
   E149 : Short_Integer; pragma Import (Ada, E149, "system__storage_pools_E");
   E151 : Short_Integer; pragma Import (Ada, E151, "system__storage_pools__subpools_E");
   E172 : Short_Integer; pragma Import (Ada, E172, "ada__calendar_E");
   E251 : Short_Integer; pragma Import (Ada, E251, "ada__calendar__delays_E");
   E184 : Short_Integer; pragma Import (Ada, E184, "ada__calendar__time_zones_E");
   E113 : Short_Integer; pragma Import (Ada, E113, "ada__text_io_E");
   E237 : Short_Integer; pragma Import (Ada, E237, "gnat__secure_hashes_E");
   E239 : Short_Integer; pragma Import (Ada, E239, "gnat__secure_hashes__sha1_E");
   E235 : Short_Integer; pragma Import (Ada, E235, "gnat__sha1_E");
   E094 : Short_Integer; pragma Import (Ada, E094, "ada__strings__maps_E");
   E192 : Short_Integer; pragma Import (Ada, E192, "ada__strings__maps__constants_E");
   E075 : Short_Integer; pragma Import (Ada, E075, "ada__strings__unbounded_E");
   E226 : Short_Integer; pragma Import (Ada, E226, "system__pool_global_E");
   E244 : Short_Integer; pragma Import (Ada, E244, "gnat__sockets_E");
   E247 : Short_Integer; pragma Import (Ada, E247, "gnat__sockets__poll_E");
   E257 : Short_Integer; pragma Import (Ada, E257, "gnat__sockets__thin_common_E");
   E249 : Short_Integer; pragma Import (Ada, E249, "gnat__sockets__thin_E");
   E201 : Short_Integer; pragma Import (Ada, E201, "system__regexp_E");
   E180 : Short_Integer; pragma Import (Ada, E180, "ada__directories_E");
   E131 : Short_Integer; pragma Import (Ada, E131, "backup_E");
   E137 : Short_Integer; pragma Import (Ada, E137, "bytes_E");
   E178 : Short_Integer; pragma Import (Ada, E178, "codec_E");
   E261 : Short_Integer; pragma Import (Ada, E261, "flow_E");
   E263 : Short_Integer; pragma Import (Ada, E263, "json_E");
   E265 : Short_Integer; pragma Import (Ada, E265, "monitor_E");
   E230 : Short_Integer; pragma Import (Ada, E230, "msgpack_E");
   E224 : Short_Integer; pragma Import (Ada, E224, "layout_E");
   E267 : Short_Integer; pragma Import (Ada, E267, "picture_E");
   E259 : Short_Integer; pragma Import (Ada, E259, "table_E");
   E232 : Short_Integer; pragma Import (Ada, E232, "websocket_E");
   E170 : Short_Integer; pragma Import (Ada, E170, "plug_E");
   E159 : Short_Integer; pragma Import (Ada, E159, "chan_E");
   E269 : Short_Integer; pragma Import (Ada, E269, "schema_E");
   E273 : Short_Integer; pragma Import (Ada, E273, "selfmap_E");
   E271 : Short_Integer; pragma Import (Ada, E271, "zone_E");

   Sec_Default_Sized_Stacks : array (1 .. 1) of aliased System.Secondary_Stack.SS_Stack (System.Parameters.Runtime_Default_Sec_Stack_Size);

   Local_Priority_Specific_Dispatching : constant String := "";
   Local_Interrupt_States : constant String := "";

   Is_Elaborated : Boolean := False;

   procedure finalize_library is
   begin
      E271 := E271 - 1;
      declare
         procedure F1;
         pragma Import (Ada, F1, "zone__finalize_spec");
      begin
         F1;
      end;
      E273 := E273 - 1;
      declare
         procedure F2;
         pragma Import (Ada, F2, "selfmap__finalize_spec");
      begin
         F2;
      end;
      E269 := E269 - 1;
      declare
         procedure F3;
         pragma Import (Ada, F3, "schema__finalize_spec");
      begin
         F3;
      end;
      E170 := E170 - 1;
      declare
         procedure F4;
         pragma Import (Ada, F4, "plug__finalize_spec");
      begin
         F4;
      end;
      E259 := E259 - 1;
      declare
         procedure F5;
         pragma Import (Ada, F5, "table__finalize_spec");
      begin
         F5;
      end;
      E267 := E267 - 1;
      declare
         procedure F6;
         pragma Import (Ada, F6, "picture__finalize_spec");
      begin
         F6;
      end;
      E224 := E224 - 1;
      declare
         procedure F7;
         pragma Import (Ada, F7, "layout__finalize_spec");
      begin
         F7;
      end;
      E230 := E230 - 1;
      declare
         procedure F8;
         pragma Import (Ada, F8, "msgpack__finalize_spec");
      begin
         F8;
      end;
      E263 := E263 - 1;
      declare
         procedure F9;
         pragma Import (Ada, F9, "json__finalize_spec");
      begin
         F9;
      end;
      E137 := E137 - 1;
      declare
         procedure F10;
         pragma Import (Ada, F10, "bytes__finalize_spec");
      begin
         F10;
      end;
      declare
         procedure F11;
         pragma Import (Ada, F11, "ada__directories__finalize_body");
      begin
         E180 := E180 - 1;
         F11;
      end;
      declare
         procedure F12;
         pragma Import (Ada, F12, "ada__directories__finalize_spec");
      begin
         F12;
      end;
      E201 := E201 - 1;
      declare
         procedure F13;
         pragma Import (Ada, F13, "system__regexp__finalize_spec");
      begin
         F13;
      end;
      declare
         procedure F14;
         pragma Import (Ada, F14, "gnat__sockets__finalize_body");
      begin
         E244 := E244 - 1;
         F14;
      end;
      declare
         procedure F15;
         pragma Import (Ada, F15, "gnat__sockets__finalize_spec");
      begin
         F15;
      end;
      E226 := E226 - 1;
      declare
         procedure F16;
         pragma Import (Ada, F16, "system__pool_global__finalize_spec");
      begin
         F16;
      end;
      E075 := E075 - 1;
      declare
         procedure F17;
         pragma Import (Ada, F17, "ada__strings__unbounded__finalize_spec");
      begin
         F17;
      end;
      E235 := E235 - 1;
      declare
         procedure F18;
         pragma Import (Ada, F18, "gnat__sha1__finalize_spec");
      begin
         F18;
      end;
      E113 := E113 - 1;
      declare
         procedure F19;
         pragma Import (Ada, F19, "ada__text_io__finalize_spec");
      begin
         F19;
      end;
      E151 := E151 - 1;
      declare
         procedure F20;
         pragma Import (Ada, F20, "system__storage_pools__subpools__finalize_spec");
      begin
         F20;
      end;
      E220 := E220 - 1;
      declare
         procedure F21;
         pragma Import (Ada, F21, "ada__streams__stream_io__finalize_spec");
      begin
         F21;
      end;
      declare
         procedure F22;
         pragma Import (Ada, F22, "system__file_io__finalize_body");
      begin
         E119 := E119 - 1;
         F22;
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
      E008 := E008 + 1;
      System.Exceptions'Elab_Spec;
      E017 := E017 + 1;
      System.Soft_Links.Initialize'Elab_Body;
      E048 := E048 + 1;
      E010 := E010 + 1;
      E014 := E014 + 1;
      Ada.Assertions'Elab_Spec;
      E135 := E135 + 1;
      Ada.Containers'Elab_Spec;
      E005 := E005 + 1;
      Ada.Io_Exceptions'Elab_Spec;
      E090 := E090 + 1;
      Ada.Numerics'Elab_Spec;
      E160 := E160 + 1;
      Ada.Strings'Elab_Spec;
      E051 := E051 + 1;
      Ada.Strings.Utf_Encoding'Elab_Spec;
      E055 := E055 + 1;
      Gnat'Elab_Spec;
      E233 := E233 + 1;
      Interfaces.C'Elab_Spec;
      E080 := E080 + 1;
      Interfaces.C.Strings'Elab_Spec;
      E205 := E205 + 1;
      System.Os_Lib'Elab_Body;
      E123 := E123 + 1;
      Ada.Tags'Elab_Spec;
      Ada.Tags'Elab_Body;
      E063 := E063 + 1;
      Ada.Strings.Text_Buffers'Elab_Spec;
      E053 := E053 + 1;
      Ada.Streams'Elab_Spec;
      E089 := E089 + 1;
      System.File_Control_Block'Elab_Spec;
      E129 := E129 + 1;
      System.Finalization_Root'Elab_Spec;
      E092 := E092 + 1;
      Ada.Finalization'Elab_Spec;
      E087 := E087 + 1;
      System.File_Io'Elab_Body;
      E119 := E119 + 1;
      Ada.Streams.Stream_Io'Elab_Spec;
      E220 := E220 + 1;
      System.Storage_Pools'Elab_Spec;
      E149 := E149 + 1;
      System.Storage_Pools.Subpools'Elab_Spec;
      E151 := E151 + 1;
      Ada.Calendar'Elab_Spec;
      Ada.Calendar'Elab_Body;
      E172 := E172 + 1;
      Ada.Calendar.Delays'Elab_Body;
      E251 := E251 + 1;
      Ada.Calendar.Time_Zones'Elab_Spec;
      E184 := E184 + 1;
      Ada.Text_Io'Elab_Spec;
      Ada.Text_Io'Elab_Body;
      E113 := E113 + 1;
      E237 := E237 + 1;
      E239 := E239 + 1;
      Gnat.Sha1'Elab_Spec;
      E235 := E235 + 1;
      Ada.Strings.Maps'Elab_Spec;
      E094 := E094 + 1;
      Ada.Strings.Maps.Constants'Elab_Spec;
      E192 := E192 + 1;
      Ada.Strings.Unbounded'Elab_Spec;
      E075 := E075 + 1;
      System.Pool_Global'Elab_Spec;
      E226 := E226 + 1;
      Gnat.Sockets'Elab_Spec;
      Gnat.Sockets.Thin_Common'Elab_Spec;
      E257 := E257 + 1;
      E249 := E249 + 1;
      Gnat.Sockets'Elab_Body;
      E244 := E244 + 1;
      E247 := E247 + 1;
      System.Regexp'Elab_Spec;
      E201 := E201 + 1;
      Ada.Directories'Elab_Spec;
      Ada.Directories'Elab_Body;
      E180 := E180 + 1;
      E131 := E131 + 1;
      Bytes'Elab_Spec;
      E137 := E137 + 1;
      E178 := E178 + 1;
      E261 := E261 + 1;
      Json'Elab_Spec;
      E263 := E263 + 1;
      E265 := E265 + 1;
      Msgpack'Elab_Spec;
      E230 := E230 + 1;
      Layout'Elab_Spec;
      E224 := E224 + 1;
      Picture'Elab_Spec;
      E267 := E267 + 1;
      Table'Elab_Spec;
      E259 := E259 + 1;
      E232 := E232 + 1;
      Plug'Elab_Spec;
      E170 := E170 + 1;
      E159 := E159 + 1;
      Schema'Elab_Spec;
      E269 := E269 + 1;
      Selfmap'Elab_Spec;
      E273 := E273 + 1;
      Zone'Elab_Spec;
      E271 := E271 + 1;
   end adainit;

   procedure Ada_Main_Program;
   pragma Import (Ada, Ada_Main_Program, "_ada_selfcheck");

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
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/flow.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/json.o
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
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/zone.o
   --   /Users/gogogod/Project/phi/body-driver/driver/obj/selfcheck.o
   --   -L/Users/gogogod/Project/phi/body-driver/driver/obj/
   --   -L/Users/gogogod/Project/phi/body-driver/driver/obj/
   --   -L/users/gogogod/.local/share/alire/toolchains/gnat_native_16.1.0_657cf254/lib/gcc/aarch64-apple-darwin24.6.0/16.1.0/adalib/
   --   -static
   --   -lgnat
   --   -lm
--  END Object file/option list   

end ada_main;
